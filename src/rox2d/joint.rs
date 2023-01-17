use std::rc::Rc;

use super::{
    body::Body,
    math::{Mat2x2, Vec2},
    world::World,
};

#[derive(Debug, Clone, Copy)]
pub struct Joint {
    /// Mass matrix (M)
    mass_matrix: Mat2x2,

    local_anchor1: Vec2,
    local_anchor2: Vec2,

    r1: Vec2,
    r2: Vec2,
    bias: Vec2,

    /// Accumulated impulse (P)
    accumulated_impulse: Vec2,
    bias_factor: f32,
    softness: f32,
}

impl Joint {
    pub fn new(body1: &mut Body, body2: &mut Body, anchor: Vec2) -> Self {
        let rot1 = Mat2x2::from_angle(body1.rotation);
        let rot2 = Mat2x2::from_angle(body2.rotation);
        let rot1_t = rot1.transpose();
        let rot2_t = rot2.transpose();

        let local_anchor1 = rot1_t * (anchor - body1.position);
        let local_anchor2 = rot2_t * (anchor - body2.position);

        Self {
            mass_matrix: Mat2x2::ZERO,
            local_anchor1,
            local_anchor2,
            r1: Vec2::ZERO,
            r2: Vec2::ZERO,
            bias: Vec2::ZERO,
            accumulated_impulse: Vec2::ZERO,
            bias_factor: 0.2,
            softness: 0.0,
        }
    }

    pub fn pre_step(
        &mut self,
        body1: &mut Body,
        body2: &mut Body,
        inv_dt: f32,
    ) {
        // Pre-compute anchors, mass matrix, and bias.
        let rot1 = Mat2x2::from_angle(body1.rotation);
        let rot2 = Mat2x2::from_angle(body2.rotation);

        self.r1 = rot1 * self.local_anchor1;
        self.r2 = rot2 * self.local_anchor2;

        // deltaV = deltaV0 + K * impulse
        // invM = [(1/m1 + 1/m2) * eye(2)
        //         - skew(r1) * invI1 * skew(r1)
        //         - skew(r2) * invI2 * skew(r2)]
        //
        //      = [1/m1+1/m2     0    ]
        //         + invI1 * [r1.y*r1.y -r1.x*r1.y]
        //         + invI2 * [r1.y*r1.y -r1.x*r1.y]
        //
        //        [    0     1/m1+1/m2]
        //        [-r1.x*r1.y r1.x*r1.x]
        //        [-r1.x*r1.y r1.x*r1.x]
        let body1_inv_i = body1.inv_moment_of_inertia;
        let body2_inv_i = body2.inv_moment_of_inertia;
        let k1 = Mat2x2::new(
            body1.inv_mass + body2.inv_mass,
            0.0,
            0.0,
            body1.inv_mass + body2.inv_mass,
        );
        let k2 = Mat2x2::new(
            body1_inv_i * self.r1.y * self.r1.y,
            -body1_inv_i * self.r1.x * self.r1.y,
            -body1_inv_i * self.r1.x * self.r1.y,
            body1_inv_i * self.r1.x * self.r1.x,
        );
        let k3 = Mat2x2::new(
            body2_inv_i * self.r2.y * self.r2.y,
            -body2_inv_i * self.r2.x * self.r2.y,
            -body2_inv_i * self.r2.x * self.r2.y,
            body2_inv_i * self.r2.x * self.r2.x,
        );
        let k = k1 + k2 + k3 + Mat2x2::from_diag(Vec2::splat(self.softness));

        self.mass_matrix = k.invert();

        let p1 = body1.position + self.r1;
        let p2 = body2.position + self.r2;
        let dp = p2 - p1;

        if World::POSITION_CORRECTION {
            self.bias = -self.bias_factor * inv_dt * dp;
        } else {
            self.bias = Vec2::ZERO;
        }

        if World::WARM_STARTING {
            // Apply accumulated impulse.
            body1.velocity -= body1.inv_mass * self.accumulated_impulse;
            body1.angular_velocity -= body1.inv_moment_of_inertia
                * self.r1.cross(self.accumulated_impulse);

            body2.velocity += body2.inv_mass * self.accumulated_impulse;
            body2.angular_velocity += body2.inv_moment_of_inertia
                * self.r2.cross(self.accumulated_impulse);
        } else {
            self.accumulated_impulse = Vec2::ZERO;
        }
    }

    pub fn apply_impulse(&mut self, body1: &mut Body, body2: &mut Body) {
        let dv = body2.velocity
            + Vec2::scalar_cross(body2.angular_velocity, self.r2)
            - body1.velocity
            - Vec2::scalar_cross(body1.angular_velocity, self.r1);

        let impulse = self.mass_matrix
            * (-dv - self.bias - self.softness * self.accumulated_impulse);

        body1.velocity -= body1.inv_mass * impulse;
        body1.angular_velocity -=
            body1.inv_moment_of_inertia * self.r1.cross(impulse);

        body2.velocity += body2.inv_mass * impulse;
        body2.angular_velocity +=
            body2.inv_moment_of_inertia * self.r2.cross(impulse);

        self.accumulated_impulse += impulse;
    }
}
