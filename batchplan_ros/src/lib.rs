//! batchplan for ROS 2: MoveIt planning requests in, timed trajectories out, without MoveIt.

mod goals;
mod limits;
mod planner;
mod scene;
mod srdf;
mod state;
mod trajectory;

pub use limits::parse as parse_joint_limits;
pub use planner::{Answer, DeviceKind, Planner, PlannerOptions};

/// Why a request was refused, as a MoveIt error code and a message.
pub(crate) struct Refusal {
    pub code: i32,
    pub message: String,
}

impl Refusal {
    pub fn new(code: i32, message: impl Into<String>) -> Self {
        Self { code, message: message.into() }
    }
}
