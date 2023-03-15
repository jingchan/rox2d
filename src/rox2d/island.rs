use crate::{
    body::{Body, BodyFlags, BodyType},
    common::{
        Vec2, ANGULAR_SLEEP_TOLERANCE, LINEAR_SLEEP_TOLERANCE, TIME_TO_SLEEP,
    },
    joint::Joint,
    time_step::{Position, Velocity},
};

use super::{
    common::{
        MAX_ROTATION, MAX_ROTATION_SQUARED, MAX_TRANSLATION,
        MAX_TRANSLATION_SQUARED,
    },
    contact::Contact,
    contact_solver::{ContactSolver, ContactSolverDef},
    time_step::{SolverData, TimeStep},
};

#[derive(Clone, Copy, Debug)]
struct IntermediateData {
    pub pos: Position,
    pub vel: Velocity,
}

pub struct Island {
    // allocator: &'static mut dyn Allocator,
    // listener: Option<&'static dyn ContactListener>,
    pub bodies: Vec<Body>,
    pub contacts: Vec<Contact>,
    pub joints: Vec<Joint>,

    // Internal ephemeral data
    data: Vec<IntermediateData>,
}

impl Island {
    pub fn new(
        initial_body_capacity: usize,
        initial_contact_capacity: usize,
        initial_joint_capacity: usize,
        // allocator: &'static mut dyn Allocator,
        // contact_listener: Option<&'static dyn ContactListener>,
    ) -> Self {
        Island {
            bodies: Vec::with_capacity(initial_body_capacity),
            contacts: Vec::with_capacity(initial_contact_capacity),
            joints: Vec::with_capacity(initial_joint_capacity),

            data: Vec::with_capacity(initial_body_capacity),
        }
    }

    fn add_body(&mut self, body: Body) {
        self.bodies.push(body);
    }
    fn add_contact(&mut self, contact: Contact) {
        self.contacts.push(contact);
    }
    fn add_joint(&mut self, joint: Joint) {
        self.joints.push(joint);
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
        self.contacts.clear();
        self.joints.clear();
    }

    pub fn solve(&self, step: TimeStep, gravity: Vec2, allow_sleep: bool) {
        let h = step.dt;
        for (body, data) in self.bodies.iter_mut().zip(self.data.iter_mut()) {
            let c = body.sweep.c;
            let a = body.sweep.a;
            let v = body.linear_velocity;
            let w = body.angular_velocity;

            body.sweep.c0 = body.sweep.c;
            body.sweep.a0 = body.sweep.a;

            if body.body_type == BodyType::Dynamic {
                // Integrate velocities.
                v += h
                    * body.inv_mass
                    * (body.gravity_scale * body.mass * gravity + body.force);
                w += h * body.inv_inertia * body.torque;

                // Apply damping.
                // ODE: dv/dt + c * v = 0
                // Solution: v(t) = v0 * exp(-c * t)
                // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                // v2 = exp(-c * dt) * v1
                // Pade approximation:
                // v2 = v1 * 1 / (1 + c * dt)
                v *= 1.0 / (1.0 + h * body.linear_damping);
                w *= 1.0 / (1.0 + h * body.angular_damping);
            }

            // waht is this needed for?
            *data = IntermediateData {
                pos: Position { c, a },
                vel: Velocity { v, w },
            };
        }

        // Solver data
        let solver_data = SolverData {
            step,
            positions: self
                .data
                .iter()
                .map(|data| Position {
                    c: data.pos.c,
                    a: data.pos.a,
                })
                .collect(),
            velocities: self
                .data
                .iter()
                .map(|data| Velocity {
                    v: data.vel.v,
                    w: data.vel.w,
                })
                .collect(),
        };

        // Initialize velocity constraints.
        let contact_solver_def = ContactSolverDef {
            step,
            contacts: &self.contacts,
            positions: &solver_data.positions,
            velocities: &solver_data.velocities,
        };

        let contact_solver = ContactSolver::new(&contact_solver_def);
        contact_solver.initialize_velocity_constraints();

        if step.warm_starting {
            contact_solver.warm_start();
        }

        for joint in self.joints.iter() {
            joint.init_velocity_constraints(&solver_data);
        }

        // Solve velocity constraints
        for _ in 0..step.velocity_iterations {
            for joint in self.joints.iter() {
                joint.solve_velocity_constraints(&solver_data);
            }

            contact_solver.solve_velocity_constraints();
        }

        // Store impulses for warm starting
        contact_solver.store_impulses();

        // Integrate positions
        for (body, data) in self.bodies.iter_mut().zip(self.data.iter_mut()) {
            let c = data.pos.c;
            let a = data.pos.a;
            let mut v = data.vel.v;
            let mut w = data.vel.w;

            // Check for large velocities
            let translation = h * v;
            if translation.dot(translation) > MAX_TRANSLATION_SQUARED {
                let ratio = MAX_TRANSLATION / translation.length();
                v *= ratio;
            }

            let rotation = h * w;
            if rotation * rotation > MAX_ROTATION_SQUARED {
                let ratio = MAX_ROTATION / rotation.abs();
                w *= ratio;
            }

            // Integrate
            c += h * v;
            a += h * w;

            *data = IntermediateData {
                pos: Position { c, a },
                vel: Velocity { v, w },
            };
        }

        // Solve position constraints
        let mut position_solved = false;
        for _ in 0..step.position_iterations {
            let contacts_ok = contact_solver.solve_position_constraints();

            let mut joints_ok = true;
            for joint in self.joints.iter() {
                let joint_ok = joint.solve_position_constraints(&solver_data);
                joints_ok = joints_ok && joint_ok;
            }

            if contacts_ok && joints_ok {
                // Exit early if the position errors are small.
                position_solved = true;
                break;
            }
        }

        // Copy state buffers back to the bodies
        for (body, data) in self.bodies.iter_mut().zip(self.data.iter_mut()) {
            body.sweep.c = data.pos.c;
            body.sweep.a = data.pos.a;
            body.linear_velocity = data.vel.v;
            body.angular_velocity = data.vel.w;
            body.synchronize_transform();
        }

        // self.report(contact_solver.velocity_constraints());

        if allow_sleep {
            let min_sleep_time = f32::MAX;

            let mut lin_tol_sqr =
                LINEAR_SLEEP_TOLERANCE * LINEAR_SLEEP_TOLERANCE;
            let mut ang_tol_sqr =
                ANGULAR_SLEEP_TOLERANCE * ANGULAR_SLEEP_TOLERANCE;

            for body in self.bodies.iter() {
                if body.ty == BodyType::Static {
                    continue;
                }

                if (body.flags & BodyFlags::AWAKE) == BodyFlags::AWAKE
                    && body.time_of_sleep > 0.0
                {
                    body.time_of_sleep = 0.0;
                }

                if (body.flags & BodyFlags::AWAKE) == BodyFlags::AWAKE
                    && (body.linear_velocity.dot(body.linear_velocity)
                        > lin_tol_sqr
                        || body.angular_velocity * body.angular_velocity
                            > ang_tol_sqr)
                {
                    body.time_of_sleep = 0.0;
                    min_sleep_time = 0.0;
                } else {
                    body.time_of_sleep += step.dt;
                    min_sleep_time = min_sleep_time.min(body.time_of_sleep);
                }
            }

            if min_sleep_time >= TIME_TO_SLEEP {
                for body in self.bodies.iter_mut() {
                    body.set_awake(false);
                }
            }
        }
    }
}
