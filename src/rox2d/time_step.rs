use crate::common::Vec2;

pub struct TimeStep {
    pub dt: f32,
    pub inv_dt: f32,
    pub dt_ratio: f32,
    pub velocity_iterations: i32,
    pub position_iterations: i32,
    pub warm_starting: bool,
}

impl TimeStep {
    pub fn new(
        dt: f32,
        velocity_iterations: i32,
        position_iterations: i32,
        inv_dt0: f32,
        warm_starting: bool,
    ) -> Self {
        TimeStep {
            dt,
            inv_dt: if dt > 0.0 { 1.0 / dt } else { 0.0 },
            dt_ratio: dt * inv_dt0,
            velocity_iterations,
            position_iterations,
            warm_starting,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Position {
    pub c: Vec2,
    pub a: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Velocity {
    pub v: Vec2,
    pub w: f32,
}

pub struct SolverData {
    pub step: TimeStep,
    pub positions: Vec<Position>,
    pub velocities: Vec<Velocity>,
}
