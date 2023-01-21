use super::Vec2;

pub struct TimeStep {
    pub dt: f32,
    pub inv_dt: f32,
    pub dt_ratio: f32,
    pub velocity_iterations: i32,
    pub position_iterations: i32,
    pub warm_starting: bool,
}

pub struct Position {
    c: Vec2,
    a: f32,
}

pub struct Velocity {
    v: Vec2,
    w: f32,
}

pub struct SolverData {
    pub step: TimeStep,
    pub positions: Vec<Position>,
    pub velocities: Vec<Velocity>,
}
