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
        _ => bytes[..bytes.len().min(1024)]
            .windows(8)
            .any(|s| s == b"<COLLADA"),
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
                let url = node
                    .attribute("url")
                    .context("Collada instance_geometry lacks url")?;
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

// Intersect each edge with the other triangle, allowing only their shared simplex.
// Clipping also covers coplanar overlap, including adjacent faces folded onto each other.
fn faces_cross(mesh: &TriMesh, a: u32, b: u32) -> bool {
    let ia = mesh.indices()[a as usize];
    let ib = mesh.indices()[b as usize];
    let pa = ia.map(|i| mesh.vertices()[i as usize]);
    let pb = ib.map(|i| mesh.vertices()[i as usize]);
    let shared: Vec<_> = ia
        .iter()
        .filter(|i| ib.contains(i))
        .map(|&i| mesh.vertices()[i as usize])
        .collect();
    let scale = pa
        .iter()
        .chain(&pb)
        .map(|p| p.distance(pa[0]))
        .fold(0.0_f64, f64::max);
    let epsilon = scale * 1e-10;
    let allowed = |p: DVec3| match shared.as_slice() {
        [] => false,
        [v] => p.distance(*v) <= epsilon,
        [v, w] => {
            let edge = *w - *v;
            let t = ((p - *v).dot(edge) / edge.length_squared()).clamp(0.0, 1.0);
            p.distance(*v + t * edge) <= epsilon
        }
        _ => false,
    };
    for (edges, face) in [(pa, pb), (pb, pa)] {
        let normal = (face[1] - face[0]).cross(face[2] - face[0]).normalize();
        for k in 0..3 {
            let start = edges[k];
            let delta = edges[(k + 1) % 3] - start;
            let d0 = normal.dot(start - face[0]);
            let slope = normal.dot(delta);
            let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
            if slope.abs() > epsilon {
                lo = -d0 / slope;
                hi = lo;
                if !(0.0..=1.0).contains(&lo) {
                    continue;
                }
            } else if d0.abs() > epsilon {
                continue;
            }
            for j in 0..3 {
                let inward = normal.cross(face[(j + 1) % 3] - face[j]).normalize();
                let distance = inward.dot(start - face[j]);
                let rate = inward.dot(delta);
                if rate.abs() <= epsilon {
                    if distance < -epsilon {
                        hi = -1.0;
                        break;
                    }
                } else if rate > 0.0 {
                    lo = lo.max(-distance / rate);
                } else {
                    hi = hi.min(-distance / rate);
                }
            }
            if lo <= hi + 1e-12
                && hi >= 0.0
                && lo <= 1.0
                && (!allowed(start + delta * lo.clamp(0.0, 1.0))
                    || !allowed(start + delta * hi.clamp(0.0, 1.0)))
            {
                return true;
            }
        }
    }
    false
}

pub(crate) fn solid_parts(vertices: Vec<DVec3>, triangles: Vec<[u32; 3]>) -> Result<Vec<TriMesh>> {
    ensure!(!triangles.is_empty(), "mesh has no triangles");
    ensure!(
        vertices.iter().all(|v| v.is_finite()),
        "mesh contains non-finite vertices"
    );
    ensure!(
        triangles
            .iter()
            .flatten()
            .all(|&i| (i as usize) < vertices.len()),
        "mesh contains out-of-range vertex indices"
    );
    // Planning-scene meshes, like STL files, may repeat vertices per face.
    // Weld exact positions before interpreting adjacency and shared intersections.
    let mut positions = HashMap::new();
    let mut welded = vec![];
    let remap: Vec<u32> = vertices
        .into_iter()
        .map(|p| {
            let key = p.to_array().map(|v| if v == 0.0 { 0 } else { v.to_bits() });
            *positions.entry(key).or_insert_with(|| {
                let index = welded.len() as u32;
                welded.push(p);
                index
            })
        })
        .collect();
    let vertices = welded;
    let triangles: Vec<_> = triangles
        .into_iter()
        .map(|f| f.map(|i| remap[i as usize]))
        .collect();
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
    let mut crossing = false;
    mesh.bvh()
        .traverse_bvtt_single_tree::<false>(&mut Default::default(), &mut |a, b| {
            crossing = crossing || faces_cross(&mesh, a, b);
        });
    ensure!(
        !crossing,
        "world collision mesh contains self-intersections"
    );
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
            let slot = mesh.indices()[edge.face as usize]
                .iter()
                .position(|&v| v == edge.vertex)
                .expect("half-edge vertex belongs to its face");
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
    for (vertices, mut faces) in mesh
        .connected_components()
        .context("mesh components were not computed")?
        .to_mesh_buffers(&mesh)
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
    urdf: &str,
    robot_links: &HashSet<String>,
    directories: &[PathBuf],
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

#[cfg(test)]
mod tests {
    use super::*;

    fn cube() -> (Vec<DVec3>, Vec<[u32; 3]>) {
        let vertices = vec![
            DVec3::new(-1., -1., -1.),
            DVec3::new(1., -1., -1.),
            DVec3::new(1., 1., -1.),
            DVec3::new(-1., 1., -1.),
            DVec3::new(-1., -1., 1.),
            DVec3::new(1., -1., 1.),
            DVec3::new(1., 1., 1.),
            DVec3::new(-1., 1., 1.),
        ];
        let faces = vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ];
        (vertices, faces)
    }

    #[test]
    fn closed_mesh_rejects_corner_pushed_through_opposite_face() {
        let (mut vertices, faces) = cube();
        assert!(solid_parts(vertices.clone(), faces.clone()).is_ok());
        vertices[6] = DVec3::new(0.2, 0.3, -2.);
        assert!(
            solid_parts(vertices, faces)
                .unwrap_err()
                .to_string()
                .contains("self-intersections")
        );
    }

    #[test]
    fn mesh_rejects_open_and_degenerate_faces() {
        let (vertices, mut faces) = cube();
        faces.pop();
        assert!(solid_parts(vertices.clone(), faces).is_err());
        assert!(solid_parts(vertices, vec![[0, 0, 1]]).is_err());
    }

    #[test]
    fn face_intersection_allows_shared_edges_but_rejects_coplanar_overlap() {
        let mesh = TriMesh::new(
            vec![
                DVec3::ZERO,
                DVec3::X,
                DVec3::Y,
                -DVec3::Y,
                DVec3::new(0.2, 0.2, 0.),
            ],
            vec![[0, 1, 2], [1, 0, 3], [1, 0, 4]],
        )
        .unwrap();
        assert!(!faces_cross(&mesh, 0, 1));
        assert!(faces_cross(&mesh, 0, 2));
    }

    #[test]
    fn overlapping_shells_are_rejected_and_disjoint_shells_are_accepted() {
        let (vertices, faces) = cube();
        for (offset, valid) in [(DVec3::splat(0.5), false), (DVec3::splat(3.), true)] {
            let mut both = vertices.clone();
            both.extend(vertices.iter().map(|p| *p + offset));
            let mut triangles = faces.clone();
            triangles.extend(faces.iter().map(|f| f.map(|i| i + 8)));
            assert_eq!(solid_parts(both, triangles).is_ok(), valid);
        }
    }
    #[test]
    fn inward_shells_and_shells_touching_at_one_vertex_remain_valid() {
        let (vertices, mut faces) = cube();
        for face in &mut faces {
            face.swap(1, 2);
        }
        assert!(solid_parts(vertices.clone(), faces.clone()).is_ok());
        let mut both = vertices.clone();
        both.extend(
            vertices
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != 0)
                .map(|(_, p)| *p + DVec3::splat(2.)),
        );
        let mut triangles = faces.clone();
        triangles.extend(
            faces
                .iter()
                .map(|f| f.map(|i| if i == 0 { 6 } else { i + 7 })),
        );
        assert_eq!(solid_parts(both, triangles).unwrap().len(), 2);
    }

    #[test]
    fn nested_shells_and_invalid_indices_are_rejected() {
        let (vertices, faces) = cube();
        let mut both = vertices.clone();
        both.extend(vertices.iter().map(|p| *p * 0.5));
        let mut triangles = faces.clone();
        triangles.extend(faces.iter().map(|f| f.map(|i| i + 8)));
        assert!(
            solid_parts(both, triangles)
                .unwrap_err()
                .to_string()
                .contains("nested")
        );
        assert!(solid_parts(vertices, vec![[0, 1, 99]]).is_err());
    }

    #[test]
    fn world_primitives_keep_their_poses_and_robot_geometry_is_excluded() {
        let urdf = r#"<robot name="cell">
          <link name="arm"><collision><geometry><sphere radius="1"/></geometry></collision></link>
          <link name="wall"><collision><origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>
            <geometry><box size="2 4 6"/></geometry></collision></link>
        </robot>"#;
        let robot = HashSet::from(["arm".to_string()]);
        let (links, markers) =
            world_model(urdf, Some(&robot), &[], &SdfOptions::default()).unwrap();
        assert_eq!(links.len(), 1);
        assert_eq!(links[0].name, "wall");
        match links[0].obstacles[0] {
            Obstacle::Cuboid {
                center,
                rotation,
                half_extents,
            } => {
                assert_eq!(center, batchplan::glam::Vec3::new(1., 2., 3.));
                assert_eq!(half_extents, batchplan::glam::Vec3::new(1., 2., 3.));
                assert!(
                    (rotation * batchplan::glam::Vec3::X - batchplan::glam::Vec3::Y).length()
                        < 1e-6
                );
            }
            _ => panic!("expected a cuboid"),
        }
        assert!(!markers.is_empty());
        assert!(
            world_model(urdf, None, &[], &SdfOptions::default())
                .unwrap()
                .0
                .is_empty()
        );
        let invalid = urdf.replace("2 4 6", "0 4 6");
        assert!(world_model(&invalid, Some(&robot), &[], &SdfOptions::default()).is_err());
    }
    #[test]
    fn per_face_vertices_are_welded_before_validation() {
        let (vertices, faces) = cube();
        let duplicated: Vec<_> = faces
            .iter()
            .flatten()
            .map(|&i| vertices[i as usize])
            .collect();
        let faces = (0..faces.len() as u32)
            .map(|i| [3 * i, 3 * i + 1, 3 * i + 2])
            .collect();
        assert_eq!(solid_parts(duplicated, faces).unwrap().len(), 1);
    }
}
