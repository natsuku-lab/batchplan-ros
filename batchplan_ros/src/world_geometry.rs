//! URDF collision geometry retained as native world obstacles.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, Result, bail, ensure};
use batchplan::glam::{DQuat, DVec3, EulerRot};
use batchplan::{Obstacle, SdfGrid, SdfOptions};
use mesh_loader::Loader;
use parry3d_f64::query::PointQuery;
use parry3d_f64::shape::{TriMesh, TriMeshFlags};
use roxmltree::{Document, Node};

const MAX_MESH_FILE_BYTES: u64 = 64 * 1024 * 1024;

enum Geometry {
    Box(DVec3),
    Sphere(f64),
    Cylinder { radius: f64, half: f64 },
    Capsule { radius: f64, half: f64 },
    Mesh(Vec<TriMesh>),
}

struct Shape {
    geometry: Geometry,
    translation: DVec3,
    rotation: DQuat,
}

fn vector(text: Option<&str>, default: DVec3) -> Result<DVec3> {
    let Some(text) = text else { return Ok(default) };
    let values = text
        .split_whitespace()
        .map(str::parse::<f64>)
        .collect::<Result<Vec<_>, _>>()?;
    ensure!(
        values.len() == 3 && values.iter().all(|v| v.is_finite()),
        "invalid geometry vector '{text}'"
    );
    Ok(DVec3::new(values[0], values[1], values[2]))
}

fn positive(node: Node, name: &str) -> Result<f64> {
    let value = node
        .attribute(name)
        .with_context(|| format!("geometry lacks {name}"))?
        .parse::<f64>()?;
    ensure!(
        value.is_finite() && value > 0.0,
        "geometry {name} must be finite and positive"
    );
    Ok(value)
}

fn mesh_path(filename: &str, directories: &[PathBuf]) -> Result<PathBuf> {
    if let Some(rest) = filename.strip_prefix("package://") {
        let (package, file) = rest.split_once('/').context("invalid package mesh URI")?;
        for directory in directories {
            for path in [directory.join(package).join(file), directory.join(file)] {
                if path.is_file()
                    && (directory.file_name().is_some_and(|n| n == package)
                        || path.starts_with(directory.join(package)))
                {
                    return Ok(path);
                }
            }
        }
        bail!("cannot resolve mesh '{filename}'");
    }
    let path = PathBuf::from(filename.strip_prefix("file://").unwrap_or(filename));
    ensure!(
        path.is_absolute() && path.is_file(),
        "mesh path must resolve to an existing absolute file: {filename}"
    );
    Ok(path)
}

fn load_mesh(path: &Path, scale: DVec3) -> Result<Vec<TriMesh>> {
    ensure!(
        std::fs::metadata(path)?.len() <= MAX_MESH_FILE_BYTES,
        "world collision mesh exceeds the 64 MiB file limit: {}",
        path.display()
    );
    let bytes = std::fs::read(path)?;
    // Match mesh-loader's extension/content detection, including extensionless Collada files.
    let collada = match path.extension().and_then(|s| s.to_str()) {
        Some("dae" | "DAE") => true,
        Some("obj" | "OBJ" | "stl" | "STL") => false,
        _ => bytes[..bytes.len().min(1024)].windows(8).any(|s| s == b"<COLLADA"),
    };
    if collada {
        let doc = Document::parse(std::str::from_utf8(&bytes)?)
            .with_context(|| format!("reading Collada scene {}", path.display()))?;
        let mut instances = HashSet::new();
        for node in doc.descendants().filter(Node::is_element) {
            ensure!(
                !node.has_tag_name("instance_node"),
                "Collada instance_node is unsupported for world collisions; bake scene instances into unique meshes: {}",
                path.display()
            );
            if node.has_tag_name("instance_geometry") {
                let url = node.attribute("url").context("Collada instance_geometry lacks url")?;
                ensure!(
                    instances.insert(url),
                    "repeated Collada geometry instance '{url}' is unsupported for world collisions; bake scene instances into unique meshes: {}",
                    path.display()
                );
            }
        }
    }
    let scene = Loader::default()
        .load_from_slice(&bytes, path)
        .with_context(|| format!("loading {}", path.display()))?;
    let mut vertices = vec![];
    let mut triangles = vec![];
    let mut indices = HashMap::new();
    for mesh in scene.meshes {
        let mut remap = vec![];
        for v in mesh.vertices {
            let p = DVec3::new(f64::from(v[0]), f64::from(v[1]), f64::from(v[2])) * scale;
            ensure!(p.is_finite(), "mesh contains non-finite vertices");
            let key = p.to_array().map(|v| if v == 0.0 { 0 } else { v.to_bits() });
            let index = *indices.entry(key).or_insert_with(|| {
                let index = vertices.len() as u32;
                vertices.push(p);
                index
            });
            remap.push(index);
        }
        for face in mesh.faces {
            triangles.push(face.map(|i| remap[i as usize]));
        }
    }
    solid_parts(vertices, triangles)
        .with_context(|| format!("building solid world mesh {}", path.display()))
}

fn solid_parts(vertices: Vec<DVec3>, triangles: Vec<[u32; 3]>) -> Result<Vec<TriMesh>> {
    ensure!(!triangles.is_empty(), "mesh has no triangles");
    for &[a, b, c] in &triangles {
        ensure!(
            a != b && b != c && a != c,
            "mesh contains degenerate triangles"
        );
        let (pa, pb, pc) = (
            vertices[a as usize],
            vertices[b as usize],
            vertices[c as usize],
        );
        ensure!(
            (pb - pa).cross(pc - pa).length_squared() > 0.0,
            "mesh contains zero-area triangles"
        );
    }
    let mut mesh = TriMesh::new(vertices, triangles)?;
    mesh.set_flags(TriMeshFlags::HALF_EDGE_TOPOLOGY)
        .context("world collision meshes must be watertight and consistently oriented")?;
    let topology = mesh.topology().context("mesh topology was not computed")?;
    ensure!(
        topology.half_edges.iter().all(|edge| edge.twin != u32::MAX),
        "world collision meshes must be watertight and consistently oriented"
    );

    // Split vertex fans so shells touching only at a corner stay separate under
    // Parry's vertex-based connectivity and can have their winding corrected independently.
    let mut vertices = vec![];
    let mut triangles = mesh.indices().to_vec();
    let mut visited = vec![false; topology.half_edges.len()];
    for first in 0..topology.half_edges.len() {
        if visited[first] {
            continue;
        }
        let index = vertices.len() as u32;
        vertices.push(mesh.vertices()[topology.half_edges[first].vertex as usize]);
        let mut current = first;
        loop {
            visited[current] = true;
            let edge = &topology.half_edges[current];
            let slot = mesh.indices()[edge.face as usize].iter()
                .position(|&v| v == edge.vertex).expect("half-edge vertex belongs to its face");
            triangles[edge.face as usize][slot] = index;
            current = topology.half_edges[edge.twin as usize].next as usize;
            if current == first {
                break;
            }
        }
    }
    let mut mesh = TriMesh::new(vertices, triangles)?;
    mesh.set_flags(TriMeshFlags::CONNECTED_COMPONENTS)?;
    let mut parts = vec![];
    for (vertices, mut faces) in mesh.connected_components()
        .context("mesh components were not computed")?.to_mesh_buffers(&mesh)
    {
        let volume: f64 = faces
            .iter()
            .map(|&[a, b, c]| {
                vertices[a as usize].dot(vertices[b as usize].cross(vertices[c as usize]))
            })
            .sum();
        ensure!(volume.abs() > 0.0, "mesh component has no enclosed volume");
        if volume < 0.0 {
            for face in &mut faces {
                face.swap(1, 2);
            }
        }
        let mut part = TriMesh::new(vertices, faces)?;
        part.set_flags(TriMeshFlags::ORIENTED)?;
        parts.push(part);
    }
    for (i, part) in parts.iter().enumerate() {
        for (j, other) in parts.iter().enumerate() {
            ensure!(
                i == j
                    || !part
                        .vertices()
                        .iter()
                        .all(|&p| other.project_local_point(p, false).is_inside),
                "nested mesh shells require a cavity-aware solid representation"
            );
        }
    }
    Ok(parts)
}

fn shapes(
    urdf: &str, robot_links: &HashSet<String>, directories: &[PathBuf],
) -> Result<BTreeMap<String, Vec<Shape>>> {
    let doc = Document::parse(urdf)?;
    let mut links = BTreeMap::new();
    for link in doc
        .root_element()
        .children()
        .filter(|n| n.has_tag_name("link"))
    {
        let name = link.attribute("name").context("link lacks a name")?;
        if robot_links.contains(name) {
            continue;
        }
        let mut geometry = vec![];
        for collision in link.children().filter(|n| n.has_tag_name("collision")) {
            let origin = collision.children().find(|n| n.has_tag_name("origin"));
            let translation = vector(origin.and_then(|n| n.attribute("xyz")), DVec3::ZERO)?;
            let rpy = vector(origin.and_then(|n| n.attribute("rpy")), DVec3::ZERO)?;
            let rotation = DQuat::from_euler(EulerRot::ZYX, rpy.z, rpy.y, rpy.x);
            let node = collision
                .children()
                .find(|n| n.has_tag_name("geometry"))
                .and_then(|n| n.children().find(Node::is_element))
                .context("collision lacks geometry")?;
            let shape = match node.tag_name().name() {
                "box" => {
                    let size = vector(node.attribute("size"), DVec3::ZERO)?;
                    ensure!(
                        size.cmpgt(DVec3::ZERO).all(),
                        "box dimensions must be positive"
                    );
                    let half = size * 0.5;
                    Shape {
                        geometry: Geometry::Box(half),
                        translation,
                        rotation,
                    }
                }
                "sphere" => {
                    let radius = positive(node, "radius")?;
                    Shape {
                        geometry: Geometry::Sphere(radius),
                        translation,
                        rotation,
                    }
                }
                "cylinder" => {
                    let radius = positive(node, "radius")?;
                    let half = positive(node, "length")? * 0.5;
                    Shape {
                        geometry: Geometry::Cylinder { radius, half },
                        translation,
                        rotation,
                    }
                }
                "capsule" => {
                    let radius = positive(node, "radius")?;
                    let half = positive(node, "length")? * 0.5;
                    Shape {
                        geometry: Geometry::Capsule { radius, half },
                        translation,
                        rotation,
                    }
                }
                "mesh" => {
                    let path = mesh_path(
                        node.attribute("filename").context("mesh lacks filename")?,
                        directories,
                    )?;
                    let scale = vector(node.attribute("scale"), DVec3::ONE)?;
                    ensure!(
                        scale.abs().cmpgt(DVec3::ZERO).all(),
                        "mesh scale must be nonzero"
                    );
                    let mesh = load_mesh(&path, scale)?;
                    Shape {
                        geometry: Geometry::Mesh(mesh),
                        translation,
                        rotation,
                    }
                }
                other => bail!("unsupported world collision geometry: {other}"),
            };
            geometry.push(shape);
        }
        if !geometry.is_empty() {
            links.insert(name.to_string(), geometry);
        }
    }
    Ok(links)
}

pub(crate) fn world_model(
    urdf: &str,
    robot_links: Option<&HashSet<String>>,
    directories: &[PathBuf],
    sdf: &SdfOptions,
) -> Result<(
    Vec<crate::scene::WorldLink>,
    Vec<ros_env::visualization_msgs::msg::Marker>,
)> {
    let mut links = vec![];
    let mut markers = vec![];
    let Some(robot_links) = robot_links else {
        return Ok((links, markers));
    };
    ensure!(
        sdf.voxel.is_finite() && sdf.voxel > 0.0,
        "world_sdf_voxel_size must be finite and positive"
    );
    let sdf = *sdf;
    for (name, shapes) in shapes(urdf, robot_links, directories)? {
        let mut obstacles = vec![];
        let mut id = 0;
        for shape in shapes {
            let center = shape.translation.as_vec3();
            let rotation = shape.rotation.as_quat();
            ensure!(
                center.is_finite() && rotation.is_finite(),
                "world geometry pose exceeds f32 range"
            );
            let convert = |value: f64| -> Result<f32> {
                let value = value as f32;
                ensure!(
                    value.is_finite() && value > 0.0,
                    "world geometry dimension exceeds f32 range"
                );
                Ok(value)
            };
            let obstacle = match &shape.geometry {
                Geometry::Box(half) => {
                    let half_extents = half.as_vec3();
                    ensure!(
                        half_extents.is_finite()
                            && half_extents.cmpgt(batchplan::glam::Vec3::ZERO).all(),
                        "world box dimensions exceed f32 range"
                    );
                    Obstacle::Cuboid {
                        center,
                        rotation,
                        half_extents,
                    }
                }
                Geometry::Sphere(radius) => Obstacle::Sphere {
                    center,
                    radius: convert(*radius)?,
                },
                Geometry::Cylinder { radius, half } => Obstacle::Cylinder {
                    center,
                    rotation,
                    radius: convert(*radius)?,
                    half_height: convert(*half)?,
                },
                Geometry::Capsule { radius, half } => Obstacle::Capsule {
                    center,
                    rotation,
                    radius: convert(*radius)?,
                    half_length: convert(*half)?,
                },
                Geometry::Mesh(parts) => {
                    for part in parts {
                        let vertices: Vec<_> =
                            part.vertices().iter().map(|v| v.as_vec3()).collect();
                        let grid = SdfGrid::from_mesh(&vertices, part.indices(), &sdf)
                            .with_context(|| format!("building world SDF for '{name}'"))?;
                        let obstacle = Obstacle::Sdf {
                            grid: Arc::new(grid),
                            center,
                            rotation,
                        };
                        let additions =
                            crate::visualization::obstacle(&name, id, &obstacle, Some(part))?;
                        id += additions.len() as i32;
                        markers.extend(additions);
                        obstacles.push(obstacle);
                    }
                    rclrs::log_info!(
                        "batchplan_planner",
                        "world SDF for '{}' uses {} m voxels",
                        name,
                        sdf.voxel
                    );
                    continue;
                }
            };
            let additions = crate::visualization::obstacle(&name, id, &obstacle, None)?;
            id += additions.len() as i32;
            markers.extend(additions);
            obstacles.push(obstacle);
        }
        rclrs::log_info!(
            "batchplan_planner",
            "URDF world link '{}' uses {} native obstacles",
            name,
            obstacles.len()
        );
        links.push(crate::scene::WorldLink { name, obstacles });
    }
    Ok((links, markers))
}
