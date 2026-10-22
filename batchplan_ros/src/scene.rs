//! MoveIt planning scenes as batchplan worlds and attached objects, in the robot's root frame.

use std::collections::HashMap;
use std::hash::Hasher;
use std::sync::{Arc, Mutex};

use batchplan::glam::{Quat, Vec3};
use batchplan::{AttachedObject, Obstacle, Pose, Robot, SdfGrid, SdfOptions, SphereOptions, World};
use ros_env::geometry_msgs::msg as geometry;
use ros_env::moveit_msgs::msg::{
    AttachedCollisionObject, CollisionObject, MoveItErrorCodes as Code, PlanningScene, RobotState,
};
use ros_env::shape_msgs::msg::{Mesh, SolidPrimitive};

use crate::Refusal;

pub fn pose(p: &geometry::Pose) -> Pose {
    Pose {
        position: Vec3::new(
            p.position.x as f32,
            p.position.y as f32,
            p.position.z as f32,
        ),
        rotation: rotation(&p.orientation),
    }
}

fn transform(t: &geometry::Transform) -> Pose {
    let v = &t.translation;
    Pose {
        position: Vec3::new(v.x as f32, v.y as f32, v.z as f32),
        rotation: rotation(&t.rotation),
    }
}

pub fn rotation(o: &geometry::Quaternion) -> Quat {
    let q = [o.x, o.y, o.z, o.w].map(|v| v as f32);
    // An all-zero quaternion is how many clients leave an orientation unset.
    if q.iter().all(|&v| v == 0.0) {
        Quat::IDENTITY
    } else {
        Quat::from_array(q).normalize()
    }
}

/// Where named frames are in the robot's root frame, for one configuration.
pub struct Frames<'a> {
    robot: &'a Robot,
    q: &'a [f32],
    root: &'a str,
    /// Frames that are not links: virtual joints' parent frames and the scene's fixed frames.
    frames: HashMap<String, Pose>,
}

impl<'a> Frames<'a> {
    /// Each SRDF virtual joint's parent frame is placed by that joint's transform in the last of
    /// `states` that gives one (MoveIt's planning frame for a robot on a moving base), and
    /// otherwise coincides with the root link. The scene's fixed frame transforms place further
    /// frames relative to the root or those frames.
    pub fn new(
        robot: &'a Robot,
        q: &'a [f32],
        root: &'a str,
        virtual_joints: &[(String, String)],
        states: &[&RobotState],
        scene: &PlanningScene,
    ) -> Self {
        let mut frames = HashMap::new();
        for (joint, parent) in virtual_joints {
            let placed = states.iter().rev().find_map(|s| {
                let m = &s.multi_dof_joint_state;
                m.joint_names
                    .iter()
                    .position(|n| n == joint)
                    .and_then(|i| m.transforms.get(i))
            });
            let root_in_parent = placed.map_or(Pose::IDENTITY, transform);
            frames.insert(parent.clone(), root_in_parent.inverse());
        }
        for t in &scene.fixed_frame_transforms {
            let parent = if t.header.frame_id == root {
                Some(Pose::IDENTITY)
            } else {
                frames.get(&t.header.frame_id).copied()
            };
            if let Some(parent) = parent {
                frames.insert(
                    t.child_frame_id.clone(),
                    parent.mul_pose(transform(&t.transform)),
                );
            }
        }
        Self {
            robot,
            q,
            root,
            frames,
        }
    }

    pub fn pose_of(&self, frame: &str) -> Result<Pose, Refusal> {
        if frame.is_empty() || frame == self.root {
            return Ok(Pose::IDENTITY);
        }
        self.robot
            .link_pose(self.q, frame)
            .or_else(|| self.frames.get(frame).copied())
            .ok_or_else(|| {
                Refusal::new(
                    Code::FRAME_TRANSFORM_FAILURE,
                    format!("unknown frame '{frame}'"),
                )
            })
    }
}

/// Distance grids for mesh obstacles, built once per distinct mesh.
pub struct Grids {
    cache: Mutex<HashMap<u64, Arc<SdfGrid>>>,
    options: SdfOptions,
}

impl Grids {
    pub fn new(options: SdfOptions) -> Self {
        Self {
            cache: Mutex::new(HashMap::new()),
            options,
        }
    }

    fn grid(&self, id: &str, mesh: &Mesh) -> Result<Arc<SdfGrid>, Refusal> {
        let mut hash = Fnv::default();
        for v in &mesh.vertices {
            for c in [v.x, v.y, v.z] {
                hash.write_u64(c.to_bits());
            }
        }
        mesh.triangles
            .iter()
            .flat_map(|t| t.vertex_indices)
            .for_each(|i| hash.write_u32(i));
        let key = hash.finish();
        if let Some(grid) = self
            .cache
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&key)
        {
            return Ok(Arc::clone(grid));
        }
        let vertices: Vec<Vec3> = mesh
            .vertices
            .iter()
            .map(|v| Vec3::new(v.x as f32, v.y as f32, v.z as f32))
            .collect();
        let triangles: Vec<[u32; 3]> = mesh.triangles.iter().map(|t| t.vertex_indices).collect();
        let grid = SdfGrid::from_mesh(&vertices, &triangles, &self.options)
            .map(Arc::new)
            .map_err(|e| Refusal::new(Code::INVALID_OBJECT_NAME, format!("mesh of '{id}': {e}")))?;
        self.cache
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert(key, Arc::clone(&grid));
        Ok(grid)
    }
}

/// The scene's collision objects as one world. The scene is taken as the whole world, whatever
/// each object's operation says, except that removals are skipped.
pub fn world(scene: &PlanningScene, frames: &Frames, grids: &Grids) -> Result<World, Refusal> {
    let mut obstacles = vec![];
    for object in scene
        .world
        .collision_objects
        .iter()
        .filter(|o| o.operation != CollisionObject::REMOVE)
    {
        let base = frames.pose_of(&object.header.frame_id)?;
        obstacles.extend(shapes(object, base, Some(grids))?);
    }
    Ok(World { obstacles })
}

/// Objects held by the robot, as shapes in their links' frames.
pub fn attached(
    objects: &[&AttachedCollisionObject],
    frames: &Frames,
) -> Result<Vec<AttachedObject>, Refusal> {
    objects
        .iter()
        .map(|a| {
            let link = frames.pose_of(&a.link_name)?;
            let base = link
                .inverse()
                .mul_pose(frames.pose_of(&a.object.header.frame_id)?);
            Ok(AttachedObject {
                name: a.object.id.clone(),
                link: a.link_name.clone(),
                shapes: shapes(&a.object, base, None)?,
                touch_links: a.touch_links.clone(),
                spheres: SphereOptions {
                    budget: 16,
                    ..Default::default()
                },
            })
        })
        .collect()
}

/// An object's shapes, placed by `base` (where its header frame is). Meshes need `grids`.
fn shapes(
    object: &CollisionObject,
    base: Pose,
    grids: Option<&Grids>,
) -> Result<Vec<Obstacle>, Refusal> {
    let refuse = |what: &str| {
        Refusal::new(
            Code::INVALID_OBJECT_NAME,
            format!("object '{}': {what}", object.id),
        )
    };
    if !object.planes.is_empty() {
        return Err(refuse("planes are not supported"));
    }
    let origin = base.mul_pose(pose(&object.pose));
    let mut out = vec![];
    for (shape, at) in object.primitives.iter().zip(&object.primitive_poses) {
        out.push(primitive(shape, origin.mul_pose(pose(at))).map_err(|e| refuse(&e))?);
    }
    for (mesh, at) in object.meshes.iter().zip(&object.mesh_poses) {
        let grids = grids.ok_or_else(|| refuse("attached meshes are not supported"))?;
        let at = origin.mul_pose(pose(at));
        out.push(Obstacle::Sdf {
            grid: grids.grid(&object.id, mesh)?,
            center: at.position,
            rotation: at.rotation,
        });
    }
    Ok(out)
}

fn primitive(shape: &SolidPrimitive, at: Pose) -> Result<Obstacle, String> {
    let d: Vec<f32> = shape.dimensions.iter().map(|&v| v as f32).collect();
    let need = |n: usize| {
        if d.len() == n && d.iter().all(|v| v.is_finite() && *v > 0.0) {
            Ok(())
        } else {
            Err(format!(
                "primitive type {} needs {n} positive dimensions, got {d:?}",
                shape.type_
            ))
        }
    };
    let (center, rotation) = (at.position, at.rotation);
    Ok(match shape.type_ {
        SolidPrimitive::BOX => {
            need(3)?;
            Obstacle::Cuboid {
                center,
                half_extents: Vec3::new(d[0], d[1], d[2]) * 0.5,
                rotation,
            }
        }
        SolidPrimitive::SPHERE => {
            need(1)?;
            Obstacle::Sphere {
                center,
                radius: d[0],
            }
        }
        // A cone is checked as the cylinder around it, erring toward collision.
        SolidPrimitive::CYLINDER | SolidPrimitive::CONE => {
            need(2)?;
            Obstacle::Cylinder {
                center,
                rotation,
                radius: d[1],
                half_height: d[0] * 0.5,
            }
        }
        other => return Err(format!("primitive type {other} is not supported")),
    })
}

/// FNV-1a, a hash that stays the same across runs and Rust versions.
pub struct Fnv(u64);

impl Default for Fnv {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl Hasher for Fnv {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
        }
    }
}
