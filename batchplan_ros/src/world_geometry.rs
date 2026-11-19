//! A workcell URDF's fixed equipment as world obstacles: the collision geometry of the links
//! outside the robot, each obstacle in its link's frame. batchplan says which shapes and meshes
//! it can check against, when a mesh becomes a distance grid and when a world is uploaded.

use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, Result, bail, ensure};
use batchplan::glam::{EulerRot, Quat, Vec3};
use batchplan::{Obstacle, SdfGrid, SdfOptions};
use mesh_loader::Loader;
use ros_env::visualization_msgs::msg::Marker;
use roxmltree::{Document, Node};

use crate::scene::WorldLink;
use crate::visualization;

/// The obstacles of every link outside `robot_links`, and their markers for RViz. Without
/// `robot_links` the whole URDF is the robot and there are none. Meshes become distance grids
/// laid out by `sdf`; `directories` hold the packages `package://` paths name.
pub(crate) fn world_model(
    urdf: &str,
    robot_links: Option<&HashSet<String>>,
    directories: &[PathBuf],
    sdf: &SdfOptions,
) -> Result<(Vec<WorldLink>, Vec<Marker>)> {
    let (mut links, mut markers) = (vec![], vec![]);
    let Some(robot_links) = robot_links else {
        return Ok((links, markers));
    };
    let doc = Document::parse(urdf)?;
    for link in doc
        .root_element()
        .children()
        .filter(|n| n.has_tag_name("link"))
    {
        let name = link.attribute("name").context("link lacks a name")?;
        if robot_links.contains(name) {
            continue;
        }
        let mut obstacles = vec![];
        for collision in link.children().filter(|n| n.has_tag_name("collision")) {
            let (obstacle, mesh) = obstacle(collision, directories, sdf)
                .with_context(|| format!("world link '{name}'"))?;
            let mesh = mesh.as_ref().map(|(v, t)| (v.as_slice(), t.as_slice()));
            markers.extend(visualization::obstacle(
                name,
                markers.len() as i32,
                &obstacle,
                mesh,
            ));
            obstacles.push(obstacle);
        }
        if !obstacles.is_empty() {
            links.push(WorldLink {
                name: name.to_string(),
                touching: vec![vec![]; obstacles.len()],
                obstacles,
            });
        }
    }
    Ok((links, markers))
}

type Mesh = (Vec<Vec3>, Vec<[u32; 3]>);

/// One `<collision>` as an obstacle in its link's frame, with the mesh a distance grid came from.
fn obstacle(
    collision: Node,
    directories: &[PathBuf],
    sdf: &SdfOptions,
) -> Result<(Obstacle, Option<Mesh>)> {
    let origin = collision.children().find(|n| n.has_tag_name("origin"));
    let center = vector(origin.and_then(|n| n.attribute("xyz")), Vec3::ZERO)?;
    let rpy = vector(origin.and_then(|n| n.attribute("rpy")), Vec3::ZERO)?;
    let rotation = Quat::from_euler(EulerRot::ZYX, rpy.z, rpy.y, rpy.x);
    let shape = collision
        .children()
        .find(|n| n.has_tag_name("geometry"))
        .and_then(|n| n.children().find(Node::is_element))
        .context("a collision lacks geometry")?;
    let number = |name: &str| -> Result<f32> {
        let text = shape
            .attribute(name)
            .with_context(|| format!("a {} lacks {name}", shape.tag_name().name()))?;
        Ok(text.parse()?)
    };
    Ok(match shape.tag_name().name() {
        "box" => {
            let half_extents = vector(shape.attribute("size"), Vec3::ZERO)? * 0.5;
            let cuboid = Obstacle::Cuboid {
                center,
                rotation,
                half_extents,
            };
            (cuboid, None)
        }
        "sphere" => {
            let radius = number("radius")?;
            (Obstacle::Sphere { center, radius }, None)
        }
        "cylinder" => {
            let cylinder = Obstacle::Cylinder {
                center,
                rotation,
                radius: number("radius")?,
                half_height: number("length")? * 0.5,
            };
            (cylinder, None)
        }
        // `length` is the length of the segment between the caps' centers.
        "capsule" => {
            let capsule = Obstacle::Capsule {
                center,
                rotation,
                radius: number("radius")?,
                half_length: number("length")? * 0.5,
            };
            (capsule, None)
        }
        "mesh" => {
            let filename = shape
                .attribute("filename")
                .context("a mesh lacks filename")?;
            let path = mesh_path(filename, directories)?;
            let scale = vector(shape.attribute("scale"), Vec3::ONE)?;
            let (vertices, triangles) = load_mesh(&path, scale)?;
            let grid = SdfGrid::from_mesh(&vertices, &triangles, sdf)
                .with_context(|| format!("mesh {}", path.display()))?;
            let grid = Obstacle::Sdf {
                grid: Arc::new(grid),
                center,
                rotation,
            };
            (grid, Some((vertices, triangles)))
        }
        other => bail!("unsupported collision geometry: {other}"),
    })
}

fn vector(text: Option<&str>, default: Vec3) -> Result<Vec3> {
    let Some(text) = text else { return Ok(default) };
    let values = text
        .split_whitespace()
        .map(str::parse::<f32>)
        .collect::<Result<Vec<_>, _>>()?;
    ensure!(values.len() == 3, "'{text}' is not three numbers");
    Ok(Vec3::from_slice(&values))
}

/// A mesh file's path: `package://name/rest` under `directories`, or an absolute path.
fn mesh_path(filename: &str, directories: &[PathBuf]) -> Result<PathBuf> {
    let Some(rest) = filename.strip_prefix("package://") else {
        let path = PathBuf::from(filename.strip_prefix("file://").unwrap_or(filename));
        ensure!(
            path.is_absolute(),
            "mesh path {filename} is relative, and the URDF is given as text"
        );
        return Ok(path);
    };
    directories
        .iter()
        .map(|d| d.join(rest))
        .find(|p| p.is_file())
        .with_context(|| format!("cannot find mesh {filename}"))
}

/// Every mesh of a file as one, scaled.
fn load_mesh(path: &Path, scale: Vec3) -> Result<Mesh> {
    let scene = Loader::default()
        .load(path)
        .with_context(|| format!("loading mesh {}", path.display()))?;
    let (mut vertices, mut triangles) = (vec![], vec![]);
    for mesh in scene.meshes {
        let first = vertices.len() as u32;
        vertices.extend(mesh.vertices.iter().map(|&v| Vec3::from(v) * scale));
        triangles.extend(mesh.faces.iter().map(|f| f.map(|i| i + first)));
    }
    Ok((vertices, triangles))
}

#[cfg(test)]
mod tests {
    use super::*;

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
    }

    #[test]
    fn a_mesh_becomes_a_distance_grid_and_one_batchplan_refuses_is_refused() {
        let dir = std::env::temp_dir().join(format!("batchplan-ros-mesh-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let facets = |triangles: &[[[f32; 3]; 3]]| {
            let facet = |t: &[[f32; 3]; 3]| {
                let [a, b, c] = t.map(|v| format!("vertex {} {} {}\n", v[0], v[1], v[2]));
                format!("facet normal 0 0 0\nouter loop\n{a}{b}{c}endloop\nendfacet\n")
            };
            let facets: String = triangles.iter().map(facet).collect();
            format!("solid mesh\n{facets}endsolid mesh\n")
        };
        let (a, b, c, d) = ([0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]);
        let closed = [[a, c, b], [a, b, d], [b, c, d], [c, a, d]];
        let urdf = |file: &str| {
            format!(
                r#"<robot name="cell"><link name="arm"/><link name="fixture"><collision>
                <geometry><mesh filename="{}" scale="0.5 0.5 0.5"/></geometry>
                </collision></link></robot>"#,
                dir.join(file).display()
            )
        };
        let robot = HashSet::from(["arm".to_string()]);
        let model =
            |file: &str| world_model(&urdf(file), Some(&robot), &[], &SdfOptions::default());

        std::fs::write(dir.join("closed.stl"), facets(&closed)).unwrap();
        let (links, markers) = model("closed.stl").unwrap();
        let Obstacle::Sdf { grid, .. } = &links[0].obstacles[0] else {
            panic!("expected a distance grid");
        };
        // The unit tetrahedron at half its size, as scaled.
        let size = grid.dims().map(|d| (d - 1) as f32 * grid.voxel());
        assert!(size.into_iter().all(|s| s < 1.0), "{size:?}");
        assert_eq!(markers[0].points.len(), 12);

        std::fs::write(dir.join("open.stl"), facets(&closed[..3])).unwrap();
        let refused = format!("{:#}", model("open.stl").err().unwrap());
        assert!(refused.contains("world link 'fixture'"), "{refused}");
        assert!(refused.contains("not closed"), "{refused}");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
