//! RViz markers for the collision spheres actually used by the planner.

use std::collections::BTreeMap;

use anyhow::{Result, bail};
use batchplan::glam::Vec3;
use batchplan::{CollisionModel, Obstacle};
use parry3d_f64::shape::TriMesh;
use ros_env::geometry_msgs::msg::{Point, Quaternion, Vector3};
use ros_env::std_msgs::msg::{ColorRGBA, Header};
use ros_env::visualization_msgs::msg::{Marker, MarkerArray};

/// All spheres grouped by link and radius, following the robot's TF frames.
#[must_use]
pub fn spheres(model: &CollisionModel) -> MarkerArray {
    let mut markers = vec![Marker {
        action: Marker::DELETEALL,
        ..Default::default()
    }];
    for (link, spheres) in &model.spheres {
        let mut radii = BTreeMap::<u32, Vec<Point>>::new();
        for sphere in spheres {
            radii.entry(sphere[3].to_bits()).or_default().push(Point {
                x: sphere[0].into(),
                y: sphere[1].into(),
                z: sphere[2].into(),
            });
        }

        for (id, (radius, points)) in radii.into_iter().enumerate() {
            let diameter = 2.0 * f64::from(f32::from_bits(radius));
            let mut marker = Marker {
                header: Header {
                    frame_id: link.clone(),
                    ..Default::default()
                },
                ns: format!("robot/{link}"),
                id: id as i32,
                type_: Marker::SPHERE_LIST,
                action: Marker::ADD,
                scale: Vector3 {
                    x: diameter,
                    y: diameter,
                    z: diameter,
                },
                color: ColorRGBA {
                    r: 0.15,
                    g: 0.75,
                    b: 1.0,
                    a: 0.35,
                },
                frame_locked: true,
                points,
                ..Default::default()
            };
            marker.pose.orientation.w = 1.0;
            markers.push(marker);
        }
    }
    MarkerArray { markers }
}

/// Robot spheres and URDF world shapes, in their respective link frames.
#[must_use]
pub fn scene(model: &CollisionModel, world: &[Marker]) -> MarkerArray {
    let mut result = spheres(model);
    result.markers.extend_from_slice(world);
    result
}

pub(crate) fn obstacle(
    link: &str,
    id: i32,
    obstacle: &Obstacle,
    mesh: Option<&TriMesh>,
) -> Result<Vec<Marker>> {
    let at = obstacle.pose();
    let mut marker = Marker {
        header: Header {
            frame_id: link.into(),
            ..Default::default()
        },
        ns: format!("world/{link}"),
        id,
        action: Marker::ADD,
        frame_locked: true,
        color: ColorRGBA {
            r: 1.0,
            g: 0.55,
            b: 0.15,
            a: 0.35,
        },
        ..Default::default()
    };
    marker.pose.position = Point {
        x: at.position.x.into(),
        y: at.position.y.into(),
        z: at.position.z.into(),
    };
    marker.pose.orientation = Quaternion {
        x: at.rotation.x.into(),
        y: at.rotation.y.into(),
        z: at.rotation.z.into(),
        w: at.rotation.w.into(),
    };
    let scale = match obstacle {
        Obstacle::Cuboid { half_extents, .. } => {
            marker.type_ = Marker::CUBE;
            *half_extents * 2.0
        }
        Obstacle::Sphere { radius, .. } => {
            marker.type_ = Marker::SPHERE;
            Vec3::splat(2.0 * radius)
        }
        Obstacle::Cylinder {
            radius,
            half_height,
            ..
        } => {
            marker.type_ = Marker::CYLINDER;
            Vec3::new(2.0 * radius, 2.0 * radius, 2.0 * half_height)
        }
        Obstacle::Capsule {
            radius,
            half_length,
            ..
        } => {
            marker.type_ = Marker::CYLINDER;
            marker.scale = Vector3 {
                x: (2.0 * radius).into(),
                y: (2.0 * radius).into(),
                z: (2.0 * half_length).into(),
            };
            let mut markers = vec![marker.clone()];
            for (i, sign) in [-1.0, 1.0].into_iter().enumerate() {
                let position = at.position + at.rotation * Vec3::Z * (sign * half_length);
                let mut cap = marker.clone();
                cap.id += i as i32 + 1;
                cap.type_ = Marker::SPHERE;
                cap.pose.position = Point {
                    x: position.x.into(),
                    y: position.y.into(),
                    z: position.z.into(),
                };
                cap.scale = Vector3 {
                    x: (2.0 * radius).into(),
                    y: (2.0 * radius).into(),
                    z: (2.0 * radius).into(),
                };
                markers.push(cap);
            }
            return Ok(markers);
        }
        Obstacle::Sdf { .. } => {
            let mesh = mesh.ok_or_else(|| anyhow::anyhow!("SDF marker lacks source mesh"))?;
            marker.type_ = Marker::TRIANGLE_LIST;
            marker.points = mesh
                .indices()
                .iter()
                .flatten()
                .map(|&i| {
                    let p = mesh.vertices()[i as usize];
                    Point {
                        x: p.x,
                        y: p.y,
                        z: p.z,
                    }
                })
                .collect();
            Vec3::ONE
        }
        other => bail!("unsupported world visualization: {other:?}"),
    };
    marker.scale = Vector3 {
        x: scale.x.into(),
        y: scale.y.into(),
        z: scale.z.into(),
    };
    Ok(vec![marker])
}
