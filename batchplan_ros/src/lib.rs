//! batchplan for ROS 2: MoveIt planning requests in, timed trajectories out, without MoveIt.

// The library's own rules from Google's Rust style guide: documented public items, `#[must_use]`
// pure functions, and `expect` with its reason rather than `unwrap` outside tests.
#![warn(missing_docs, missing_debug_implementations)]
#![warn(clippy::must_use_candidate, clippy::return_self_not_must_use)]
#![cfg_attr(not(test), warn(clippy::unwrap_used))]

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
        Self {
            code,
            message: message.into(),
        }
    }
}
