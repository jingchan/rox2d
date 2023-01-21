use std::hash::Hash;

use super::{
    body::Body,
    collide::{self, Contact},
    math::Vec2,
    world::{self, World},
};

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ArbiterKey {
    pub body1: usize,
    pub body2: usize,
}

impl ArbiterKey {
    pub fn new(body1: usize, body2: usize) -> Self {
        // We want to make sure that body1 is always the body with the lower
        // address, so that we can use the ordering
        if body1 < body2 {
            ArbiterKey { body1, body2 }
        } else {
            ArbiterKey {
                body1: body2,
                body2: body1,
            }
        }
    }
}

const MAX_POINTS: usize = 2;
#[derive(Debug, Clone, Copy)]
pub struct Arbiter {
    pub contacts: [Contact; MAX_POINTS],
    pub num_contacts: usize,
    pub friction: f32,
}

impl Arbiter {
    pub fn new(body1: &Body, body2: &Body) -> Self {
        let (body1, body2) = if body1 < body2 {
            (body1, body2)
        } else {
            (body2, body1)
        };

        let mut contacts = [Contact::default(); MAX_POINTS];
        let num_contacts = collide::collide(&mut contacts, &body1, &body2);
        let friction = (body1.friction * body2.friction).sqrt();

        Self {
            contacts,
            num_contacts,
            friction,
        }
    }

    pub fn update(
        &mut self,
        new_contacts: &[Contact],
        num_new_contacts: usize,
    ) {
        let mut merged_contacts = [Contact::default(); MAX_POINTS];

        for i in 0..num_new_contacts {
            let c_new = new_contacts[i];
            let mut k = None;
            for j in 0..self.num_contacts {
                let c_old = self.contacts[j];
                if c_new.feature_pair == c_old.feature_pair {
                    k = Some(j);
                    break;
                }
            }

            if let Some(k) = k {
                let mut c = &mut merged_contacts[i];
                let c_old = self.contacts[k];
                *c = c_new;
                if world::World::WARM_STARTING {
                    c.accumulated_impulse_normal =
                        c_old.accumulated_impulse_normal;
                    c.accumulated_impulse_tangent =
                        c_old.accumulated_impulse_tangent;
                    c.accumulated_impulse_normal_biasnb =
                        c_old.accumulated_impulse_normal_biasnb;
                } else {
                    c.accumulated_impulse_normal = 0.0;
                    c.accumulated_impulse_tangent = 0.0;
                    c.accumulated_impulse_normal_biasnb = 0.0;
                }
            } else {
                merged_contacts[i] = self.contacts[i];
            }
        }

        for i in 0..self.num_contacts {
            self.contacts[i] = merged_contacts[i];
        }
    }

    pub fn pre_step(
        &mut self,
        body1: &mut Body,
        body2: &mut Body,
        inv_dt: f32,
    ) {
        const ALLOWED_PENETRATION: f32 = 0.01;
        let bias_factor = if world::World::POSITION_CORRECTION {
            0.2
        } else {
            0.0
        };

        for i in 0..self.num_contacts {
            let c = &mut self.contacts[i];

            let r1 = c.position - body1.position;
            let r2 = c.position - body2.position;

            // Precompute normal mass, tangent mass, and bias.
            let rn1 = r1.dot(c.normal);
            let rn2 = r2.dot(c.normal);
            let mut k_normal = body1.inv_mass + body2.inv_mass;
            k_normal += body1.inv_inertia * (r1.dot(r1) - rn1 * rn1)
                + body2.inv_inertia * (r2.dot(r2) - rn2 * rn2);

            c.mass_normal = 1.0 / k_normal;

            let tangent = c.normal.cross_scalar(1.0);
            let rt1 = r1.dot(tangent);
            let rt2 = r2.dot(tangent);
            let mut k_tangent = body1.inv_mass + body2.inv_mass;
            k_tangent += body1.inv_inertia * (r1.dot(r1) - rt1 * rt1)
                + body2.inv_inertia * (r2.dot(r2) - rt2 * rt2);
            c.mass_tangent = 1.0 / k_tangent;

            c.bias = -bias_factor
                * inv_dt
                * (c.separation + ALLOWED_PENETRATION).max(0.0);

            if World::ACCUMULATE_IMPULSES {
                // Apply normal + friction impulse
                let accumulated_impulse = c.accumulated_impulse_normal
                    * c.normal
                    + c.accumulated_impulse_tangent * tangent;

                body1.velocity -= body1.inv_mass * accumulated_impulse;
                body1.angular_velocity -=
                    body1.inv_inertia * r1.cross(accumulated_impulse);

                body2.velocity += body2.inv_mass * accumulated_impulse;
                body2.angular_velocity +=
                    body2.inv_inertia * r2.cross(accumulated_impulse);
            }
        }
    }

    pub fn apply_impulse(&mut self, body1: &mut Body, body2: &mut Body) {
        for i in 0..self.num_contacts {
            let c = &mut self.contacts[i];

            c.r1 = c.position - body1.position;
            c.r2 = c.position - body2.position;

            // Relative velocity at contact
            let dv = body2.velocity
                + Vec2::scalar_cross(body2.angular_velocity, c.r2)
                - body1.velocity
                - Vec2::scalar_cross(body1.angular_velocity, c.r1);

            // Compute normal impulse
            let vn = dv.dot(c.normal);
            let mut impulse_normal = c.mass_normal * (-vn + c.bias);

            if World::ACCUMULATE_IMPULSES {
                // Clamp the accumulated impulse
                let accumulated_impulse_normal0 = c.accumulated_impulse_normal;
                c.accumulated_impulse_normal =
                    (c.accumulated_impulse_normal + impulse_normal).max(0.0);
                impulse_normal =
                    c.accumulated_impulse_normal - accumulated_impulse_normal0;
            } else {
                impulse_normal = impulse_normal.max(0.0);
            }

            // Apply contact impulse
            let accumulated_impulse_normal = impulse_normal * c.normal;

            body1.velocity -= body1.inv_mass * accumulated_impulse_normal;
            body1.angular_velocity -=
                body1.inv_inertia * c.r1.cross(accumulated_impulse_normal);

            body2.velocity += body2.inv_mass * accumulated_impulse_normal;
            body2.angular_velocity +=
                body2.inv_inertia * c.r2.cross(accumulated_impulse_normal);

            // Relative velocity at contact
            let dv = body2.velocity
                + Vec2::scalar_cross(body2.angular_velocity, c.r2)
                - body1.velocity
                - Vec2::scalar_cross(body1.angular_velocity, c.r1);

            // Compute tangent impulse
            let tangent = c.normal.cross_scalar(1.0);
            let vt = dv.dot(tangent);
            let mut impulse_tangent = c.mass_tangent * (-vt);

            if World::ACCUMULATE_IMPULSES {
                // Compute friction impulse
                let max_accumulated_impulse_tangent =
                    self.friction * c.accumulated_impulse_normal;

                // Clamp friction
                let old_tangent_impulse = c.accumulated_impulse_tangent;
                c.accumulated_impulse_tangent =
                    (old_tangent_impulse + impulse_tangent).clamp(
                        -max_accumulated_impulse_tangent,
                        max_accumulated_impulse_tangent,
                    );
                impulse_tangent =
                    c.accumulated_impulse_tangent - old_tangent_impulse;
            } else {
                // Compute friction impulse
                let max_impulse_tangent = self.friction * impulse_normal;

                // Clamp friction
                impulse_tangent = impulse_tangent
                    .clamp(-max_impulse_tangent, max_impulse_tangent);
            }

            // Apply contact impulse
            let accumulated_impulse_tangent = impulse_tangent * tangent;

            body1.velocity -= body1.inv_mass * accumulated_impulse_tangent;
            body1.angular_velocity -=
                body1.inv_inertia * c.r1.cross(accumulated_impulse_tangent);

            body2.velocity += body2.inv_mass * accumulated_impulse_tangent;
            body2.angular_velocity +=
                body2.inv_inertia * c.r2.cross(accumulated_impulse_tangent);
        }
    }
}
