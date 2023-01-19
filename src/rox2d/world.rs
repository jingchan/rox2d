use std::collections::HashMap;

use super::{
    arbiter::{Arbiter, ArbiterKey},
    body::Body,
    joint::Joint,
    math::Vec2,
};

#[derive(Debug)]
pub struct World {
    pub bodies: Vec<Body>,
    pub joints: Vec<(usize, usize, Joint)>,
    pub arbiters: HashMap<ArbiterKey, Arbiter>,
    pub gravity: Vec2,
}

impl World {
    pub const ACCUMULATE_IMPULSES: bool = true;
    pub const WARM_STARTING: bool = true;
    pub const POSITION_CORRECTION: bool = true;

    pub fn new(gravity: Vec2) -> World {
        World {
            bodies: Vec::new(),
            joints: Vec::new(),
            arbiters: HashMap::new(),
            gravity: gravity,
        }
    }

    pub fn add_body(&mut self, mut body: Body) -> usize {
        let id = self.bodies.len();
        body.id = id;
        self.bodies.push(body);
        id
    }

    /// TODO: Take a body definition instead of a body.
    pub fn create_body(&mut self, size: Vec2, mass: f32) -> usize {
        let body = Body::new(size, mass);
        self.add_body(body)
    }

    pub fn destroy_body(&mut self, body_id: usize) {
        self.bodies.remove(body_id);

        // TODO: Iteratively destroy all joints, contacts, and fixtures and bodies attached to the body.
    }

    pub fn get_body(&mut self, body_id: usize) -> Option<&Body> {
        self.bodies.get(body_id)
    }

    /// TODO: Take a joint definition instead of a joint.
    pub fn create_joint(&mut self, body1: usize, body2: usize, joint: Joint) {
        self.joints.push((body1, body2, joint));
    }

    /// TODO:
    pub fn destroy_joint(joint: &Joint) {
        todo!()
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
        self.joints.clear();
        self.arbiters.clear();
    }

    fn broad_phase(&mut self) {
        // 	O(n^2) broad-phase
        for i in 0..self.bodies.len() {
            let bi = &self.bodies[i];

            for j in i + 1..self.bodies.len() {
                let bj = &self.bodies[j];

                if bi.inv_mass == 0.0 && bj.inv_mass == 0.0 {
                    continue;
                }

                let new_arb = Arbiter::new(bi, bj);
                let key = ArbiterKey::new(bi.id, bj.id);

                if new_arb.num_contacts > 0 {
                    if let Some(arb) = self.arbiters.get_mut(&key) {
                        arb.update(&new_arb.contacts, new_arb.num_contacts);
                    } else {
                        self.arbiters.insert(key, new_arb);
                    }
                } else {
                    self.arbiters.remove(&key);
                }
            }
        }
    }

    /// TODO: Separate velocity and position interation.
    pub fn step(&mut self, dt: f32, iterations: usize) {
        // If new fixtures were added, we need to find the new contacts.
        // if (self.has_new_contacts) {
        //     self.contact_manager.find_new_contacts();
        //     self.has_new_contacts = false;
        // }

        let inv_dt = if dt > 0.0 { 1.0 / dt } else { 0.0 };

        // Determine overlapping bodies and update contact points.
        self.broad_phase();

        // Integrate forces
        for body in &mut self.bodies {
            if body.inv_mass == 0.0 {
                continue;
            }

            body.velocity += (self.gravity + body.inv_mass * body.force) * dt;
            body.angular_velocity += body.inv_inertia * body.torque * dt;
        }

        // Perform pre-steps.
        for (key, arb) in self.arbiters.iter_mut() {
            let [body1, body2] =
                self.bodies.get_many_mut([key.body1, key.body2]).unwrap();
            arb.pre_step(body1, body2, inv_dt);
        }

        for (body1, body2, joint) in &mut self.joints {
            let [body1, body2] =
                self.bodies.get_many_mut([*body1, *body2]).unwrap();
            joint.pre_step(body1, body2, inv_dt);
        }

        // Perform iterations
        for _ in 0..iterations {
            for (key, arb) in self.arbiters.iter_mut() {
                let [body1, body2] =
                    self.bodies.get_many_mut([key.body1, key.body2]).unwrap();
                arb.apply_impulse(body1, body2);
            }

            for (body1, body2, joint) in &mut self.joints {
                let [body1, body2] =
                    self.bodies.get_many_mut([*body1, *body2]).unwrap();
                joint.apply_impulse(body1, body2);
            }
        }

        // Integrate velocities
        for body in &mut self.bodies {
            body.position += dt * body.velocity;
            body.rotation += dt * body.angular_velocity;

            body.force = Vec2::ZERO;
            body.torque = 0.0;
        }
    }
    fn clear_forces() {
        todo!()
    }
}
