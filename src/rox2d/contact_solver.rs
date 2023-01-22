use crate::{
    rox2d::common::{BAUMGARTE, LINEAR_SLOP, MAX_LINEAR_CORRECTION},
    Rot, Transform,
};

use super::{
    collision::{Manifold, ManifoldType, WorldManifold},
    common::MAX_MANIFOLD_POINTS,
    contact::Contact,
    island::{Position, Velocity},
    time_step::TimeStep,
    Body, Mat2x2, Vec2,
};

const DEBUG_SOLVER: bool = false;
const BLOCK_SOLVE: bool = true;

#[derive(Copy, Clone, Default)]
pub struct VelocityContraintPoint {
    pub r_a: Vec2,
    pub r_b: Vec2,
    pub normal_impulse: f32,
    pub tangent_impulse: f32,
    pub normal_mass: f32,
    pub tangent_mass: f32,
    pub velocity_bias: f32,
}

pub struct ContactVelocityConstraint {
    pub points: [VelocityContraintPoint; MAX_MANIFOLD_POINTS],
    normal: Vec2,
    normal_mass: Mat2x2,
    k: Mat2x2,
    index_a: i32,
    index_b: i32,
    inv_mass_a: f32,
    inv_mass_b: f32,
    inv_inertia_a: f32,
    inv_inertia_b: f32,
    friction: f32,
    restitution: f32,
    threshold: f32,
    tangent_speed: f32,
    point_count: i32,
    contact_index: i32,
}

impl ContactVelocityConstraint {
    fn new(
        body_a: &Body,
        body_b: &Body,
        step: TimeStep,
        contact: &Contact,
    ) -> Self {
        let manifold = contact.get_manifold();
        let vc_points = if step.warm_starting {
            manifold
                .points
                .iter()
                .map(|contact| {
                    let normal_impulse = step.dt_ratio * contact.normal_impulse;
                    let tangent_impulse =
                        step.dt_ratio * contact.tangent_impulse;
                    VelocityContraintPoint {
                        normal_impulse,
                        tangent_impulse,
                        ..Default::default()
                    }
                })
                .collect()
        } else {
            std::array::from_fn(|_| Default::default())
        };

        Self {
            points: vc_points,
            normal: Vec2::ZERO,
            normal_mass: Mat2x2::ZERO,
            k: Mat2x2::ZERO,
            index_a: body_a.island_index,
            index_b: body_b.island_index,
            inv_mass_a: body_a.inv_mass,
            inv_mass_b: body_b.inv_mass,
            inv_inertia_a: body_a.inv_inertia,
            inv_inertia_b: body_b.inv_inertia,
            friction: contact.friction,
            restitution: contact.restitution,
            threshold: contact.threshold,
            tangent_speed: contact.tangent_speed,
            point_count: manifold.point_count,
            contact_index: contact.index,
        }
    }
}

pub struct ContactSolverDef {
    step: TimeStep,
    contacts: Vec<Contact>,
    positions: Vec<Position>,
    velocities: Vec<Velocity>,
    // allocator: StackAllocator,
}

#[derive(Copy, Clone)]
struct ContactPositionConstraint {
    local_points: [Vec2; MAX_MANIFOLD_POINTS],
    local_normal: Vec2,
    local_point: Vec2,
    index_a: i32,
    index_b: i32,
    inv_mass_a: f32,
    inv_mass_b: f32,
    local_center_a: Vec2,
    local_center_b: Vec2,
    inv_inertia_a: f32,
    inv_inertia_b: f32,
    manifold_type: ManifoldType,
    radius_a: f32,
    radius_b: f32,
    point_count: i32,
}

impl ContactPositionConstraint {
    fn new(body_a: &Body, body_b: &Body, manifold: &Manifold) -> Self {
        Self {
            local_points: manifold
                .points
                .iter()
                .map(|p| p.local_point)
                .collect(),
            local_normal: Vec2::ZERO,
            local_point: Vec2::ZERO,
            index_a: body_a.island_index,
            index_b: body_b.island_index,
            inv_mass_a: body_a.inv_mass,
            inv_mass_b: body_b.inv_mass,
            local_center_a: body_a.sweep.local_center,
            local_center_b: body_b.sweep.local_center,
            inv_inertia_a: body_a.inv_inertia,
            inv_inertia_b: body_b.inv_inertia,
            manifold_type: manifold.manifold_type,
            radius_a: manifold.radius_a,
            radius_b: manifold.radius_b,
            point_count: manifold.point_count,
        }
    }
}

struct PositionSolverManifold {
    normal: Vec2,
    point: Vec2,
    separation: f32,
}

impl PositionSolverManifold {
    fn initialize(
        &mut self,
        pc: &ContactPositionConstraint,
        xf_a: &Transform,
        xf_b: &Transform,
        index: i32,
    ) {
        assert!(pc.point_count > 0);

        match pc.type_ {
            ManifoldType::Circles => {
                let point_a = Transform::mul_vec2(xf_a, pc.local_point);
                let point_b = Transform::mul_vec2(xf_b, pc.local_points[0]);
                let normal = (point_b - point_a).normalize();
                self.normal = normal;
                self.point = 0.5 * (point_a + point_b);
                self.separation = (point_b - point_a).dot(self.normal)
                    - pc.radius_a
                    - pc.radius_b;
            }

            ManifoldType::FaceA => {
                self.normal = Transform::mul_rot(xf_a.q, pc.local_normal);
                let plane_point = Transform::mul_vec2(xf_a, pc.local_point);

                let clip_point =
                    Transform::mul_vec2(xf_b, pc.local_points[index]);
                self.separation = (clip_point - plane_point).dot(self.normal)
                    - pc.radius_a
                    - pc.radius_b;
                self.point = clip_point;
            }

            ManifoldType::FaceB => {
                self.normal = Transform::mul_rot(xf_b.q, pc.local_normal);
                let plane_point = Transform::mul_vec2(xf_b, pc.local_point);

                let clip_point =
                    Transform::mul_vec2(xf_a, pc.local_points[index]);
                self.separation = (clip_point - plane_point).dot(self.normal)
                    - pc.radius_a
                    - pc.radius_b;
                self.point = clip_point;

                // Ensure normal points from A to B
                self.normal = -self.normal;
            }
        }
    }
}

pub struct ContactSolver {
    step: TimeStep,
    positions: Vec<Position>,
    velocities: Vec<Velocity>,
    // allocator: StackAllocator,
    position_constraints: Vec<ContactPositionConstraint>,
    velocity_constraints: Vec<ContactVelocityConstraint>,
    contacts: Vec<Contact>,
}

impl ContactSolver {
    pub fn new(def: ContactSolverDef) {
        let mut solver = ContactSolver {
            step: def.step,
            positions: def.positions,
            velocities: def.velocities,
            // allocator: def.allocator,
            position_constraints: Vec::new(),
            velocity_constraints: Vec::new(),
            contacts: def.contacts,
        };
        solver.init_position_indpendent_contraints(def.step);
        return solver;
    }

    fn init_position_indpendent_contraints(&mut self, step: TimeStep) {
        for contact in self.contacts.iter() {
            let fixture_a = contact.fixture_a;
            let fixture_b = contact.fixture_b;
            let shape_a = fixture_a.shape;
            let shape_b = fixture_b.shape;
            let radius_a = shape_a.radius;
            let radius_b = shape_b.radius;
            let body_a = fixture_a.body;
            let body_b = fixture_b.body;
            let manifold = contact.get_manifold();

            let point_count = manifold.point_count;

            let vc_points = std::array::from_fn(|i| {
                if step.warm_starting {
                    VelocityContraintPoint {
                        normal_impulse: contact.normal_impulse,
                        tangent_impulse: contact.tangent_impulse,
                        ..Default::default()
                    }
                } else {
                    VelocityContraintPoint::default()
                }
            });

            for i in 0..point_count {
                self.velocity_constraints[i] = ContactVelocityConstraint::new(
                    &body_a, &body_b, step, contact,
                );
                self.position_constraints[i] =
                    ContactPositionConstraint::new(&body_a, &body_b, &manifold);
            }
        }
    }

    /// Initialize position dependent portions of the velocity constraints.
    fn initialize_velocity_constraints(&self) {
        for (vc, pc) in self
            .velocity_constraints
            .iter_mut()
            .zip(self.position_constraints.iter())
        {
            let radius_a = pc.radius_a;
            let radius_b = pc.radius_b;
            let manifold = self.contacts[vc.contact_index].get_manifold();

            let index_a = vc.index_a;
            let index_b = vc.index_b;

            let local_center_a = pc.local_center_a;
            let local_center_b = pc.local_center_b;

            let m_a = vc.inv_mass_a;
            let m_b = vc.inv_mass_b;
            let i_a = vc.inv_inertia_a;
            let i_b = vc.inv_inertia_b;

            let c_a = self.positions[index_a].c;
            let a_a = self.positions[index_a].a;
            let v_a = self.velocities[index_a].v;
            let w_a = self.velocities[index_a].w;

            let c_b = self.positions[index_b].c;
            let a_b = self.positions[index_b].a;
            let v_b = self.velocities[index_b].v;
            let w_b = self.velocities[index_b].w;

            let xf_a = Transform::new(a_a, c_a);
            let xf_b = Transform::new(a_b, c_b);

            let world_manifold = WorldManifold::initialize(
                &manifold, &xf_a, radius_a, &xf_b, radius_b,
            );

            vc.normal = world_manifold.normal;

            let point_count = vc.point_count;
            for i in 0..point_count {
                let cp = manifold.points[i];
                let vcp = vc.points[i];

                vcp.r_a = world_manifold.points[i] - c_a;
                vcp.r_b = world_manifold.points[i] - c_b;

                let rnA = vcp.r_a.cross(vc.normal);
                let rnB = vcp.r_b.cross(vc.normal);

                let kNormal = m_a + m_b + i_a * rnA * rnA + i_b * rnB * rnB;

                vcp.normal_mass =
                    if kNormal > 0.0 { 1.0 / kNormal } else { 0.0 };

                let tangent = vc.normal.cross(1.0);

                let rtA = vcp.r_a.cross(tangent);
                let rtB = vcp.r_b.cross(tangent);

                let kTangent = m_a + m_b + i_a * rtA * rtA + i_b * rtB * rtB;

                vcp.tangent_mass =
                    if kTangent > 0.0 { 1.0 / kTangent } else { 0.0 };

                // Setup a velocity bias for restitution.
                vcp.velocity_bias = 0.0;
                let vRel = vc.normal.dot(
                    v_b + vc.normal.cross(w_b) - v_a - vc.normal.cross(w_a),
                );
                if vRel < -vc.threshold {
                    vcp.velocity_bias = -vc.restitution * vRel;
                }
            }

            // If we have two points, then prepare the block solver.
            if vc.point_count == 2 && BLOCK_SOLVE {
                let vcp1 = vc.points[0];
                let vcp2 = vc.points[1];

                let rn1A = vcp1.rA.cross(vc.normal);
                let rn1B = vcp1.rB.cross(vc.normal);
                let rn2A = vcp2.rA.cross(vc.normal);
                let rn2B = vcp2.rB.cross(vc.normal);

                let k11 = m_a + m_b + i_a * rn1A * rn1A + i_b * rn1B * rn1B;
                let k22 = m_a + m_b + i_a * rn2A * rn2A + i_b * rn2B * rn2B;
                let k12 = m_a + m_b + i_a * rn1A * rn2A + i_b * rn1B * rn2B;

                // Ensure a reasonable condition number.
                let k_max_condition_number = 1000.0;
                if k11 * k11 < k_max_condition_number * (k11 * k22 - k12 * k12)
                {
                    // K is safe to invert.
                    vc.K.ex = Vec2::new(k11, k12);
                    vc.K.ey = Vec2::new(k12, k22);
                    vc.normal_mass = vc.K.inverse();
                } else {
                    // The constraints are redundant, just use one.
                    // TODO_ERIN use deepest?
                    vc.point_count = 1;
                }
            }
        }
    }

    pub fn warm_start(&mut self) {
        // Warm start.
        for i in 0..self.count {
            let vc = &self.velocity_constraints[i];

            let index_a = vc.index_a;
            let index_b = vc.index_b;
            let m_a = vc.inv_mass_a;
            let i_a = vc.inv_inertia_a;
            let m_b = vc.inv_mass_b;
            let i_b = vc.inv_inertia_b;
            let point_count = vc.point_count;

            let mut v_a = self.velocities[index_a].v;
            let mut w_a = self.velocities[index_a].w;
            let mut v_b = self.velocities[index_b].v;
            let mut w_b = self.velocities[index_b].w;

            for j in 0..point_count {
                let vcp = vc.points[j];
                let P = vcp.normal_impulse * vc.normal
                    + vcp.tangent_impulse * vc.normal.cross(1.0);
                w_a -= i_a * vcp.r_a.cross(P);
                v_a -= m_a * P;
                w_b += i_b * vcp.r_b.cross(P);
                v_b += m_b * P;
            }

            self.velocities[index_a].v = v_a;
            self.velocities[index_a].w = w_a;
            self.velocities[index_b].v = v_b;
            self.velocities[index_b].w = w_b;
        }
    }

    pub fn solve_velocity_constraints(&mut self) {
        for i in 0..self.count {
            let vc = &self.velocity_constraints[i];

            let index_a = vc.index_a;
            let index_b = vc.index_b;
            let m_a = vc.inv_mass_a;
            let i_a = vc.inv_inertia_a;
            let m_b = vc.inv_mass_b;
            let i_b = vc.inv_inertia_b;
            let point_count = vc.point_count;

            let mut v_a = self.velocities[index_a].v;
            let mut w_a = self.velocities[index_a].w;
            let mut v_b = self.velocities[index_b].v;
            let mut w_b = self.velocities[index_b].w;

            let tangent = vc.normal.cross(1.0);

            // Solve tangent constraints first because non-penetration is more
            // important than friction.
            for j in 0..point_count {
                let mut vcp = vc.points[j];

                // Relative velocity at contact
                let dv = v_b + w_b.cross(vcp.r_b) - v_a - w_a.cross(vcp.r_a);

                // Compute tangent force
                let vt = dv.dot(tangent) - vc.tangent_speed;
                let lambda = vcp.tangent_mass * (-vt);

                // b2Clamp the accumulated force
                let max_friction = vc.friction * vcp.normal_impulse;
                let new_impulse = (vcp.tangent_impulse + lambda)
                    .clamp(-max_friction, max_friction);
                lambda = new_impulse - vcp.tangent_impulse;
                vcp.tangent_impulse = new_impulse;

                // Apply contact impulse
                let P = lambda * vc.tangent;

                v_a -= m_a * P;
                w_a -= i_a * vcp.r_a.cross(P);

                v_b += m_b * P;
                w_b += i_b * vcp.r_b.cross(P);
            }

            // Solve normal constraints
            if vc.point_count == 1 || !BLOCK_SOLVE {
                for j in 0..point_count {
                    let mut vcp = vc.points[j];

                    // Relative velocity at contact
                    let dv =
                        v_b + w_b.cross(vcp.r_b) - v_a - w_a.cross(vcp.r_a);

                    // Compute normal impulse
                    let vn = dv.dot(vc.normal);
                    let mut lambda =
                        -vcp.normal_mass * (vn - vcp.velocity_bias);

                    // b2Clamp the accumulated impulse
                    let new_impulse = (vcp.normal_impulse + lambda).max(0.0);
                    lambda = new_impulse - vcp.normal_impulse;
                    vcp.normal_impulse = new_impulse;

                    // Apply contact impulse
                    let P = lambda * vc.normal;

                    v_a -= m_a * P;
                    w_a -= i_a * vcp.r_a.cross(P);

                    v_b += m_b * P;
                    w_b += i_b * vcp.r_b.cross(P);
                }
            } else {
                // Block solver developed in collaboration with Dirk Gregorius
                // (back in 01/07 on Box2D_Lite).  Build the mini LCP for this
                // contact patch
                //
                // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i =
                // 1..2
                //
                // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n ) b = vn0 -
                // velocityBias
                //
                // The system is solved using the "Total enumeration method" (s.
                // Murty). The complementary constraint vn_i * x_i implies that
                // we must have in any solution either vn_i = 0 or x_i = 0. So
                // for the 2D contact problem the cases vn_1 = 0 and x_1 = 0 or
                // vn_2 = 0 and x_2 = 0, x_1 = 0 and vn_2 = 0 and vn_1 = 0 need
                // to be tested. The first valid solution that satisfies the
                // problem is chosen.
                //
                // In order to account of the accumulated impulse 'a' (because
                // of the iterative nature of the solver which only requires
                // that the accumulated impulse is clamped and not the
                // incremental impulse) we change the impulse variable (x_i).
                //
                // Substitute:
                //
                // x = a + d
                //
                // a := old total impulse
                // x := new total impulse
                // d := incremental impulse
                //
                // For the current iteration we extend the formula for the
                // incremental impulse
                // to compute the new total impulse:
                //
                // vn = A * d + b
                //    = A * (x - a) + b
                //    = A * x + b - A * a
                //    = A * x + b'
                // b' = b - A * a;

                let mut cp1 = vc.points[0];
                let mut cp2 = vc.points[1];

                let a = Vec2::new(cp1.normal_impulse, cp2.normal_impulse);
                debug_assert!(a.x >= 0.0 && a.y >= 0.0);

                // Relative velocity at contact
                let dv1 = v_b + w_b.cross(cp1.r_b) - v_a - w_a.cross(cp1.r_a);
                let dv2 = v_b + w_b.cross(cp2.r_b) - v_a - w_a.cross(cp2.r_a);

                // Compute normal velocity
                let vn1 = dv1.dot(vc.normal);
                let vn2 = dv2.dot(vc.normal);

                let mut b =
                    Vec2::new(vn1 - cp1.velocity_bias, vn2 - cp2.velocity_bias);

                // Compute b'
                b -= vc.k * a;

                loop {
                    //
                    // Case 1: vn = 0
                    //
                    // 0 = A * x + b'
                    //
                    // Solve for x:
                    //
                    // x = - inv(A) * b'
                    //
                    let x = -vc.normal_mass * b;

                    if x.x >= 0.0 && x.y >= 0.0 {
                        // Get the incremental impulse
                        let d = x - a;

                        // Apply incremental impulse
                        let P1 = d.x * vc.normal;
                        let P2 = d.y * vc.normal;
                        v_a -= m_a * (P1 + P2);
                        w_a -= i_a * (cp1.r_a.cross(P1) + cp2.r_a.cross(P2));

                        v_b += m_b * (P1 + P2);
                        w_b += i_b * (cp1.r_b.cross(P1) + cp2.r_b.cross(P2));

                        // Accumulate
                        cp1.normal_impulse = x.x;
                        cp2.normal_impulse = x.y;

                        break;
                    }

                    //
                    // Case 2: vn1 = 0 and x2 = 0
                    //
                    //   0 = a11 * x1 + a12 * 0 + b1'
                    // vn2 = a21 * x1 + a22 * 0 + b2'
                    //
                    x.x = -cp1.normal_mass * b.x;
                    x.y = 0.0;
                    vn1 = 0.0;
                    vn2 = vc.k.ex.y * x.x + b.y;

                    if x.x >= 0.0 && vn2 >= 0.0 {
                        // Get the incremental impulse
                        let d = x - a;

                        // Apply incremental impulse
                        let P1 = d.x * vc.normal;
                        let P2 = d.y * vc.normal;
                        v_a -= m_a * (P1 + P2);
                        w_a -= i_a * (cp1.r_a.cross(P1) + cp2.r_a.cross(P2));

                        v_b += m_b * (P1 + P2);
                        w_b += i_b * (cp1.r_b.cross(P1) + cp2.r_b.cross(P2));

                        // Accumulate
                        cp1.normal_impulse = x.x;
                        cp2.normal_impulse = x.y;

                        break;
                    }

                    //
                    // Case 3: vn2 = 0 and x1 = 0
                    //
                    // vn1 = a11 * 0 + a12 * x2 + b1'
                    //   0 = a21 * 0 + a22 * x2 + b2'
                    //
                    x.x = 0.0;
                    x.y = -cp2.normal_mass * b.y;
                    vn1 = vc.k.ey.x * x.y + b.x;
                    vn2 = 0.0;

                    if x.y >= 0.0 && vn1 >= 0.0 {
                        // Resubstitute for the incremental impulse
                        let d = x - a;

                        // Apply incremental impulse
                        let P1 = d.x * vc.normal;
                        let P2 = d.y * vc.normal;
                        v_a -= m_a * (P1 + P2);
                        w_a -= i_a * (cp1.r_a.cross(P1) + cp2.r_a.cross(P2));

                        v_b += m_b * (P1 + P2);
                        w_b += i_b * (cp1.r_b.cross(P1) + cp2.r_b.cross(P2));

                        // Accumulate
                        cp1.normal_impulse = x.x;
                        cp2.normal_impulse = x.y;

                        break;
                    }

                    //
                    // Case 4: x1 = 0 and x2 = 0
                    //
                    // vn1 = b1
                    // vn2 = b2;
                    x.x = 0.0;
                    x.y = 0.0;
                    vn1 = b.x;
                    vn2 = b.y;

                    if vn1 >= 0.0 && vn2 >= 0.0 {
                        // Resubstitute for the incremental impulse
                        let d = x - a;

                        // Apply incremental impulse
                        let P1 = d.x * vc.normal;
                        let P2 = d.y * vc.normal;
                        v_a -= m_a * (P1 + P2);
                        w_a -= i_a * (cp1.r_a.cross(P1) + cp2.r_a.cross(P2));

                        v_b += m_b * (P1 + P2);
                        w_b += i_b * (cp1.r_b.cross(P1) + cp2.r_b.cross(P2));

                        // Accumulate
                        cp1.normal_impulse = x.x;
                        cp2.normal_impulse = x.y;

                        break;
                    }

                    // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                    break;
                }
            }

            self.velocities[index_a].v = v_a;
            self.velocities[index_a].w = w_a;
            self.velocities[index_b].v = v_b;
            self.velocities[index_b].w = w_b;
        }
    }

    pub fn store_impulses(&mut self) {
        for i in 0..self.count {
            let vc = &self.velocity_constraints[i];
            let manifold = self.contacts[vc.contact_index].get_manifold();

            for j in 0..vc.point_count {
                manifold.points[j].normal_impulse = vc.points[j].normal_impulse;
                manifold.points[j].tangent_impulse =
                    vc.points[j].tangent_impulse;
            }
        }
    }

    pub fn solve_position_contraints(&mut self) -> bool {
        let mut min_separation = 0.0;

        for i in 0..self.count {
            let pc = &self.position_constraints[i];

            let index_a = pc.index_a;
            let index_b = pc.index_b;
            let local_center_a = pc.local_center_a;
            let m_a = pc.inv_mass_a;
            let i_a = pc.inv_ia;
            let local_center_b = pc.local_center_b;
            let m_b = pc.inv_mass_b;
            let i_b = pc.inv_ib;
            let point_count = pc.point_count;

            let mut c_a = self.positions[index_a].c;
            let mut a_a = self.positions[index_a].a;

            let mut c_b = self.positions[index_b].c;
            let mut a_b = self.positions[index_b].a;

            // Solve normal constraints
            for j in 0..point_count {
                let q = Rot::new(a_a);
                let xf_a = Transform {
                    p: c_a - q * local_center_a,
                    q,
                };
                let q = Rot::new(a_b);
                let xf_b = Transform {
                    p: c_b - q * local_center_b,
                    q,
                };

                let mut psm = PositionSolverManifold::new();
                psm.initialize(pc, &xf_a, &xf_b, j as i32);

                let normal = psm.normal;
                let point = psm.point;
                let separation = psm.separation;

                let r_a = point - c_a;
                let r_b = point - c_b;

                // Track max constraint error.
                min_separation = min_separation.min(separation);

                // Prevent large corrections and allow slop.
                let C = (BAUMGARTE * (separation + LINEAR_SLOP))
                    .clamp(-MAX_LINEAR_CORRECTION, 0.0);

                // Compute the effective mass.
                let rn_a = r_a.cross(normal);
                let rn_b = r_b.cross(normal);
                let K = pc.inv_mass_a
                    + pc.inv_mass_b
                    + pc.inv_i_a * rn_a * rn_a
                    + pc.inv_i_b * rn_b * rn_b;

                // Compute normal impulse
                let impulse = if K > 0.0 { -C / K } else { 0.0 };

                let P = impulse * normal;

                c_a -= pc.inv_mass_a * P;
                a_a -= pc.inv_i_a * r_a.cross(P);

                c_b += pc.inv_mass_b * P;
                a_b += pc.inv_i_b * r_b.cross(P);
            }

            self.positions[index_a].c = c_a;
            self.positions[index_a].a = a_a;

            self.positions[index_b].c = c_b;
            self.positions[index_b].a = a_b;
        }

        // We can't expect minSpeparation >= -LINEAR_SLOP because we don't
        // push the separation above -LINEAR_SLOP.
        return min_separation >= -3.0 * LINEAR_SLOP;
    }

    pub fn solve_toi_position_contraints(
        &mut self,
        toi_index_a: i32,
        toi_index_b: i32,
    ) -> bool {
        let mut min_separation = 0.0;

        for i in 0..self.count {
            let pc = &self.position_constraints[i];

            let index_a = pc.index_a;
            let index_b = pc.index_b;
            let local_center_a = pc.local_center_a;
            let local_center_b = pc.local_center_b;
            let point_count = pc.point_count;

            let m_a = if index_a == toi_index_a || index_a == toi_index_b {
                pc.inv_mass_a
            } else {
                0.0
            };
            let i_a = if index_a == toi_index_a || index_a == toi_index_b {
                pc.inv_ia
            } else {
                0.0
            };

            let mut c_a = self.positions[index_a].c;
            let mut a_a = self.positions[index_a].a;

            let mut c_b = self.positions[index_b].c;
            let mut a_b = self.positions[index_b].a;

            // Solve normal constraints
            for j in 0..point_count {
                let q = Rot::new(a_a);
                let xf_a = Transform {
                    p: c_a - q * local_center_a,
                    q,
                };
                let q = Rot::new(a_b);
                let xf_b = Transform {
                    p: c_b - q * local_center_b,
                    q,
                };

                let mut psm = PositionSolverManifold::new();
                psm.initialize(pc, &xf_a, &xf_b, j as i32);

                let normal = psm.normal;
                let point = psm.point;
                let separation = psm.separation;

                let r_a = point - c_a;
                let r_b = point - c_b;

                // Track max constraint error.
                min_separation = min_separation.min(separation);

                // Prevent large corrections and allow slop.
                let C = (BAUMGARTE * (separation + LINEAR_SLOP))
                    .clamp(-MAX_LINEAR_CORRECTION, 0.0);

                // Compute the effective mass.
                let rn_a = r_a.cross(normal);
                let rn_b = r_b.cross(normal);
                let K = pc.inv_mass_a
                    + pc.inv_mass_b
                    + pc.inv_i_a * rn_a * rn_a
                    + pc.inv_i_b * rn_b * rn_b;

                // Compute normal impulse
                let impulse = if K > 0.0 { -C / K } else { 0.0 };

                let P = impulse * normal;

                c_a -= pc.inv_mass_a * P;
                a_a -= pc.inv_i_a * r_a.cross(P);

                c_b += pc.inv_mass_b * P;
                a_b += pc.inv_i_b * r_b.cross(P);
            }

            self.positions[index_a].c = c_a;
            self.positions[index_a].a = a_a;

            self.positions[index_b].c = c_b;
            self.positions[index_b].a = a_b;
        }

        // We can't expect minSpeparation >= -LINEAR_SLOP because we don't
        // push the separation above -LINEAR_SLOP.
        return min_separation >= -1.5 * LINEAR_SLOP;
    }
}
