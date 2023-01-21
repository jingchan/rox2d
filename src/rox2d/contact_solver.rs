use crate::Transform;

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
            if vc.point_count == 2 && self.step.block_solving {
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
}

// b2ContactSolver::b2ContactSolver(b2ContactSolverDef* def)
// {
// 	m_step = def->step;k
// 	m_allocator = def->allocator;
// 	m_count = def->count;
// 	m_positionConstraints = (b2ContactPositionConstraint*)m_allocator->Allocate(m_count * sizeof(b2ContactPositionConstraint));
// 	m_velocityConstraints = (b2ContactVelocityConstraint*)m_allocator->Allocate(m_count * sizeof(b2ContactVelocityConstraint));
// 	m_positions = def->positions;
// 	m_velocities = def->velocities;
// 	m_contacts = def->contacts;

// }

// b2ContactSolver::~b2ContactSolver()
// {
// 	m_allocator->Free(m_velocityConstraints);
// 	m_allocator->Free(m_positionConstraints);
// }

// // Initialize position dependent portions of the velocity constraints.
// void b2ContactSolver::InitializeVelocityConstraints()
// {
// 	for (int32 i = 0; i < m_count; ++i)
// 	{
// 		b2ContactVelocityConstraint* vc = m_velocityConstraints + i;
// 		b2ContactPositionConstraint* pc = m_positionConstraints + i;

// 		float radiusA = pc->radiusA;
// 		float radiusB = pc->radiusB;
// 		b2Manifold* manifold = m_contacts[vc->contactIndex]->GetManifold();

// 		int32 indexA = vc->indexA;
// 		int32 indexB = vc->indexB;

// 		float mA = vc->invMassA;
// 		float mB = vc->invMassB;
// 		float iA = vc->invIA;
// 		float iB = vc->invIB;
// 		b2Vec2 localCenterA = pc->localCenterA;
// 		b2Vec2 localCenterB = pc->localCenterB;

// 		b2Vec2 cA = m_positions[indexA].c;
// 		float aA = m_positions[indexA].a;
// 		b2Vec2 vA = m_velocities[indexA].v;
// 		float wA = m_velocities[indexA].w;

// 		b2Vec2 cB = m_positions[indexB].c;
// 		float aB = m_positions[indexB].a;
// 		b2Vec2 vB = m_velocities[indexB].v;
// 		float wB = m_velocities[indexB].w;

// 		b2Assert(manifold->pointCount > 0);

// 		b2Transform xfA, xfB;
// 		xfA.q.Set(aA);
// 		xfB.q.Set(aB);
// 		xfA.p = cA - b2Mul(xfA.q, localCenterA);
// 		xfB.p = cB - b2Mul(xfB.q, localCenterB);

// 		b2WorldManifold worldManifold;
// 		worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);

// 		vc->normal = worldManifold.normal;

// 		int32 pointCount = vc->pointCount;
// 		for (int32 j = 0; j < pointCount; ++j)
// 		{
// 			b2VelocityConstraintPoint* vcp = vc->points + j;

// 			vcp->rA = worldManifold.points[j] - cA;
// 			vcp->rB = worldManifold.points[j] - cB;

// 			float rnA = b2Cross(vcp->rA, vc->normal);
// 			float rnB = b2Cross(vcp->rB, vc->normal);

// 			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

// 			vcp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

// 			b2Vec2 tangent = b2Cross(vc->normal, 1.0f);

// 			float rtA = b2Cross(vcp->rA, tangent);
// 			float rtB = b2Cross(vcp->rB, tangent);

// 			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

// 			vcp->tangentMass = kTangent > 0.0f ? 1.0f /  kTangent : 0.0f;

// 			// Setup a velocity bias for restitution.
// 			vcp->velocityBias = 0.0f;
// 			float vRel = b2Dot(vc->normal, vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA));
// 			if (vRel < -vc->threshold)
// 			{
// 				vcp->velocityBias = -vc->restitution * vRel;
// 			}
// 		}

// 		// If we have two points, then prepare the block solver.
// 		if (vc->pointCount == 2 && g_blockSolve)
// 		{
// 			b2VelocityConstraintPoint* vcp1 = vc->points + 0;
// 			b2VelocityConstraintPoint* vcp2 = vc->points + 1;

// 			float rn1A = b2Cross(vcp1->rA, vc->normal);
// 			float rn1B = b2Cross(vcp1->rB, vc->normal);
// 			float rn2A = b2Cross(vcp2->rA, vc->normal);
// 			float rn2B = b2Cross(vcp2->rB, vc->normal);

// 			float k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
// 			float k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
// 			float k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

// 			// Ensure a reasonable condition number.
// 			const float k_maxConditionNumber = 1000.0f;
// 			if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
// 			{
// 				// K is safe to invert.
// 				vc->K.ex.Set(k11, k12);
// 				vc->K.ey.Set(k12, k22);
// 				vc->normalMass = vc->K.GetInverse();
// 			}
// 			else
// 			{
// 				// The constraints are redundant, just use one.
// 				// TODO_ERIN use deepest?
// 				vc->pointCount = 1;
// 			}
// 		}
// 	}
// }

// void b2ContactSolver::WarmStart()
// {
// 	// Warm start.
// 	for (int32 i = 0; i < m_count; ++i)
// 	{
// 		b2ContactVelocityConstraint* vc = m_velocityConstraints + i;

// 		int32 indexA = vc->indexA;
// 		int32 indexB = vc->indexB;
// 		float mA = vc->invMassA;
// 		float iA = vc->invIA;
// 		float mB = vc->invMassB;
// 		float iB = vc->invIB;
// 		int32 pointCount = vc->pointCount;

// 		b2Vec2 vA = m_velocities[indexA].v;
// 		float wA = m_velocities[indexA].w;
// 		b2Vec2 vB = m_velocities[indexB].v;
// 		float wB = m_velocities[indexB].w;

// 		b2Vec2 normal = vc->normal;
// 		b2Vec2 tangent = b2Cross(normal, 1.0f);

// 		for (int32 j = 0; j < pointCount; ++j)
// 		{
// 			b2VelocityConstraintPoint* vcp = vc->points + j;
// 			b2Vec2 P = vcp->normalImpulse * normal + vcp->tangentImpulse * tangent;
// 			wA -= iA * b2Cross(vcp->rA, P);
// 			vA -= mA * P;
// 			wB += iB * b2Cross(vcp->rB, P);
// 			vB += mB * P;
// 		}

// 		m_velocities[indexA].v = vA;
// 		m_velocities[indexA].w = wA;
// 		m_velocities[indexB].v = vB;
// 		m_velocities[indexB].w = wB;
// 	}
// }

// void b2ContactSolver::SolveVelocityConstraints()
// {
// 	for (int32 i = 0; i < m_count; ++i)
// 	{
// 		b2ContactVelocityConstraint* vc = m_velocityConstraints + i;

// 		int32 indexA = vc->indexA;
// 		int32 indexB = vc->indexB;
// 		float mA = vc->invMassA;
// 		float iA = vc->invIA;
// 		float mB = vc->invMassB;
// 		float iB = vc->invIB;
// 		int32 pointCount = vc->pointCount;

// 		b2Vec2 vA = m_velocities[indexA].v;
// 		float wA = m_velocities[indexA].w;
// 		b2Vec2 vB = m_velocities[indexB].v;
// 		float wB = m_velocities[indexB].w;

// 		b2Vec2 normal = vc->normal;
// 		b2Vec2 tangent = b2Cross(normal, 1.0f);
// 		float friction = vc->friction;

// 		b2Assert(pointCount == 1 || pointCount == 2);

// 		// Solve tangent constraints first because non-penetration is more important
// 		// than friction.
// 		for (int32 j = 0; j < pointCount; ++j)
// 		{
// 			b2VelocityConstraintPoint* vcp = vc->points + j;

// 			// Relative velocity at contact
// 			b2Vec2 dv = vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA);

// 			// Compute tangent force
// 			float vt = b2Dot(dv, tangent) - vc->tangentSpeed;
// 			float lambda = vcp->tangentMass * (-vt);

// 			// b2Clamp the accumulated force
// 			float maxFriction = friction * vcp->normalImpulse;
// 			float newImpulse = b2Clamp(vcp->tangentImpulse + lambda, -maxFriction, maxFriction);
// 			lambda = newImpulse - vcp->tangentImpulse;
// 			vcp->tangentImpulse = newImpulse;

// 			// Apply contact impulse
// 			b2Vec2 P = lambda * tangent;

// 			vA -= mA * P;
// 			wA -= iA * b2Cross(vcp->rA, P);

// 			vB += mB * P;
// 			wB += iB * b2Cross(vcp->rB, P);
// 		}

// 		// Solve normal constraints
// 		if (pointCount == 1 || g_blockSolve == false)
// 		{
// 			for (int32 j = 0; j < pointCount; ++j)
// 			{
// 				b2VelocityConstraintPoint* vcp = vc->points + j;

// 				// Relative velocity at contact
// 				b2Vec2 dv = vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA);

// 				// Compute normal impulse
// 				float vn = b2Dot(dv, normal);
// 				float lambda = -vcp->normalMass * (vn - vcp->velocityBias);

// 				// b2Clamp the accumulated impulse
// 				float newImpulse = b2Max(vcp->normalImpulse + lambda, 0.0f);
// 				lambda = newImpulse - vcp->normalImpulse;
// 				vcp->normalImpulse = newImpulse;

// 				// Apply contact impulse
// 				b2Vec2 P = lambda * normal;
// 				vA -= mA * P;
// 				wA -= iA * b2Cross(vcp->rA, P);

// 				vB += mB * P;
// 				wB += iB * b2Cross(vcp->rB, P);
// 			}
// 		}
// 		else
// 		{
// 			// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
// 			// Build the mini LCP for this contact patch
// 			//
// 			// vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
// 			//
// 			// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
// 			// b = vn0 - velocityBias
// 			//
// 			// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
// 			// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
// 			// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
// 			// solution that satisfies the problem is chosen.
// 			//
// 			// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
// 			// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
// 			//
// 			// Substitute:
// 			//
// 			// x = a + d
// 			//
// 			// a := old total impulse
// 			// x := new total impulse
// 			// d := incremental impulse
// 			//
// 			// For the current iteration we extend the formula for the incremental impulse
// 			// to compute the new total impulse:
// 			//
// 			// vn = A * d + b
// 			//    = A * (x - a) + b
// 			//    = A * x + b - A * a
// 			//    = A * x + b'
// 			// b' = b - A * a;

// 			b2VelocityConstraintPoint* cp1 = vc->points + 0;
// 			b2VelocityConstraintPoint* cp2 = vc->points + 1;

// 			b2Vec2 a(cp1->normalImpulse, cp2->normalImpulse);
// 			b2Assert(a.x >= 0.0f && a.y >= 0.0f);

// 			// Relative velocity at contact
// 			b2Vec2 dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
// 			b2Vec2 dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

// 			// Compute normal velocity
// 			float vn1 = b2Dot(dv1, normal);
// 			float vn2 = b2Dot(dv2, normal);

// 			b2Vec2 b;
// 			b.x = vn1 - cp1->velocityBias;
// 			b.y = vn2 - cp2->velocityBias;

// 			// Compute b'
// 			b -= b2Mul(vc->K, a);

// 			const float k_errorTol = 1e-3f;
// 			B2_NOT_USED(k_errorTol);

// 			for (;;)
// 			{
// 				//
// 				// Case 1: vn = 0
// 				//
// 				// 0 = A * x + b'
// 				//
// 				// Solve for x:
// 				//
// 				// x = - inv(A) * b'
// 				//
// 				b2Vec2 x = - b2Mul(vc->normalMass, b);

// 				if (x.x >= 0.0f && x.y >= 0.0f)
// 				{
// 					// Get the incremental impulse
// 					b2Vec2 d = x - a;

// 					// Apply incremental impulse
// 					b2Vec2 P1 = d.x * normal;
// 					b2Vec2 P2 = d.y * normal;
// 					vA -= mA * (P1 + P2);
// 					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

// 					vB += mB * (P1 + P2);
// 					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

// 					// Accumulate
// 					cp1->normalImpulse = x.x;
// 					cp2->normalImpulse = x.y;

// #if B2_DEBUG_SOLVER == 1
// 					// Postconditions
// 					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
// 					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

// 					// Compute normal velocity
// 					vn1 = b2Dot(dv1, normal);
// 					vn2 = b2Dot(dv2, normal);

// 					b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
// 					b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
// #endif
// 					break;
// 				}

// 				//
// 				// Case 2: vn1 = 0 and x2 = 0
// 				//
// 				//   0 = a11 * x1 + a12 * 0 + b1'
// 				// vn2 = a21 * x1 + a22 * 0 + b2'
// 				//
// 				x.x = - cp1->normalMass * b.x;
// 				x.y = 0.0f;
// 				vn1 = 0.0f;
// 				vn2 = vc->K.ex.y * x.x + b.y;
// 				if (x.x >= 0.0f && vn2 >= 0.0f)
// 				{
// 					// Get the incremental impulse
// 					b2Vec2 d = x - a;

// 					// Apply incremental impulse
// 					b2Vec2 P1 = d.x * normal;
// 					b2Vec2 P2 = d.y * normal;
// 					vA -= mA * (P1 + P2);
// 					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

// 					vB += mB * (P1 + P2);
// 					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

// 					// Accumulate
// 					cp1->normalImpulse = x.x;
// 					cp2->normalImpulse = x.y;

// #if B2_DEBUG_SOLVER == 1
// 					// Postconditions
// 					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);

// 					// Compute normal velocity
// 					vn1 = b2Dot(dv1, normal);

// 					b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
// #endif
// 					break;
// 				}

// 				//
// 				// Case 3: vn2 = 0 and x1 = 0
// 				//
// 				// vn1 = a11 * 0 + a12 * x2 + b1'
// 				//   0 = a21 * 0 + a22 * x2 + b2'
// 				//
// 				x.x = 0.0f;
// 				x.y = - cp2->normalMass * b.y;
// 				vn1 = vc->K.ey.x * x.y + b.x;
// 				vn2 = 0.0f;

// 				if (x.y >= 0.0f && vn1 >= 0.0f)
// 				{
// 					// Resubstitute for the incremental impulse
// 					b2Vec2 d = x - a;

// 					// Apply incremental impulse
// 					b2Vec2 P1 = d.x * normal;
// 					b2Vec2 P2 = d.y * normal;
// 					vA -= mA * (P1 + P2);
// 					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

// 					vB += mB * (P1 + P2);
// 					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

// 					// Accumulate
// 					cp1->normalImpulse = x.x;
// 					cp2->normalImpulse = x.y;

// #if B2_DEBUG_SOLVER == 1
// 					// Postconditions
// 					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

// 					// Compute normal velocity
// 					vn2 = b2Dot(dv2, normal);

// 					b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
// #endif
// 					break;
// 				}

// 				//
// 				// Case 4: x1 = 0 and x2 = 0
// 				//
// 				// vn1 = b1
// 				// vn2 = b2;
// 				x.x = 0.0f;
// 				x.y = 0.0f;
// 				vn1 = b.x;
// 				vn2 = b.y;

// 				if (vn1 >= 0.0f && vn2 >= 0.0f )
// 				{
// 					// Resubstitute for the incremental impulse
// 					b2Vec2 d = x - a;

// 					// Apply incremental impulse
// 					b2Vec2 P1 = d.x * normal;
// 					b2Vec2 P2 = d.y * normal;
// 					vA -= mA * (P1 + P2);
// 					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

// 					vB += mB * (P1 + P2);
// 					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

// 					// Accumulate
// 					cp1->normalImpulse = x.x;
// 					cp2->normalImpulse = x.y;

// 					break;
// 				}

// 				// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
// 				break;
// 			}
// 		}

// 		m_velocities[indexA].v = vA;
// 		m_velocities[indexA].w = wA;
// 		m_velocities[indexB].v = vB;
// 		m_velocities[indexB].w = wB;
// 	}
// }

// void b2ContactSolver::StoreImpulses()
// {
// 	for (int32 i = 0; i < m_count; ++i)
// 	{
// 		b2ContactVelocityConstraint* vc = m_velocityConstraints + i;
// 		b2Manifold* manifold = m_contacts[vc->contactIndex]->GetManifold();

// 		for (int32 j = 0; j < vc->pointCount; ++j)
// 		{
// 			manifold->points[j].normalImpulse = vc->points[j].normalImpulse;
// 			manifold->points[j].tangentImpulse = vc->points[j].tangentImpulse;
// 		}
// 	}
// }

// struct b2PositionSolverManifold
// {
// 	void Initialize(b2ContactPositionConstraint* pc, const b2Transform& xfA, const b2Transform& xfB, int32 index)
// 	{
// 		b2Assert(pc->pointCount > 0);

// 		switch (pc->type)
// 		{
// 		case b2Manifold::e_circles:
// 			{
// 				b2Vec2 pointA = b2Mul(xfA, pc->localPoint);
// 				b2Vec2 pointB = b2Mul(xfB, pc->localPoints[0]);
// 				normal = pointB - pointA;
// 				normal.Normalize();
// 				point = 0.5f * (pointA + pointB);
// 				separation = b2Dot(pointB - pointA, normal) - pc->radiusA - pc->radiusB;
// 			}
// 			break;

// 		case b2Manifold::e_faceA:
// 			{
// 				normal = b2Mul(xfA.q, pc->localNormal);
// 				b2Vec2 planePoint = b2Mul(xfA, pc->localPoint);

// 				b2Vec2 clipPoint = b2Mul(xfB, pc->localPoints[index]);
// 				separation = b2Dot(clipPoint - planePoint, normal) - pc->radiusA - pc->radiusB;
// 				point = clipPoint;
// 			}
// 			break;

// 		case b2Manifold::e_faceB:
// 			{
// 				normal = b2Mul(xfB.q, pc->localNormal);
// 				b2Vec2 planePoint = b2Mul(xfB, pc->localPoint);

// 				b2Vec2 clipPoint = b2Mul(xfA, pc->localPoints[index]);
// 				separation = b2Dot(clipPoint - planePoint, normal) - pc->radiusA - pc->radiusB;
// 				point = clipPoint;

// 				// Ensure normal points from A to B
// 				normal = -normal;
// 			}
// 			break;
// 		}
// 	}

// 	b2Vec2 normal;
// 	b2Vec2 point;
// 	float separation;
// };

// // Sequential solver.
// bool b2ContactSolver::SolvePositionConstraints()
// {
// 	float minSeparation = 0.0f;

// 	for (int32 i = 0; i < m_count; ++i)
// 	{
// 		b2ContactPositionConstraint* pc = m_positionConstraints + i;

// 		int32 indexA = pc->indexA;
// 		int32 indexB = pc->indexB;
// 		b2Vec2 localCenterA = pc->localCenterA;
// 		float mA = pc->invMassA;
// 		float iA = pc->invIA;
// 		b2Vec2 localCenterB = pc->localCenterB;
// 		float mB = pc->invMassB;
// 		float iB = pc->invIB;
// 		int32 pointCount = pc->pointCount;

// 		b2Vec2 cA = m_positions[indexA].c;
// 		float aA = m_positions[indexA].a;

// 		b2Vec2 cB = m_positions[indexB].c;
// 		float aB = m_positions[indexB].a;

// 		// Solve normal constraints
// 		for (int32 j = 0; j < pointCount; ++j)
// 		{
// 			b2Transform xfA, xfB;
// 			xfA.q.Set(aA);
// 			xfB.q.Set(aB);
// 			xfA.p = cA - b2Mul(xfA.q, localCenterA);
// 			xfB.p = cB - b2Mul(xfB.q, localCenterB);

// 			b2PositionSolverManifold psm;
// 			psm.Initialize(pc, xfA, xfB, j);
// 			b2Vec2 normal = psm.normal;

// 			b2Vec2 point = psm.point;
// 			float separation = psm.separation;

// 			b2Vec2 rA = point - cA;
// 			b2Vec2 rB = point - cB;

// 			// Track max constraint error.
// 			minSeparation = b2Min(minSeparation, separation);

// 			// Prevent large corrections and allow slop.
// 			float C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

// 			// Compute the effective mass.
// 			float rnA = b2Cross(rA, normal);
// 			float rnB = b2Cross(rB, normal);
// 			float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

// 			// Compute normal impulse
// 			float impulse = K > 0.0f ? - C / K : 0.0f;

// 			b2Vec2 P = impulse * normal;

// 			cA -= mA * P;
// 			aA -= iA * b2Cross(rA, P);

// 			cB += mB * P;
// 			aB += iB * b2Cross(rB, P);
// 		}

// 		m_positions[indexA].c = cA;
// 		m_positions[indexA].a = aA;

// 		m_positions[indexB].c = cB;
// 		m_positions[indexB].a = aB;
// 	}

// 	// We can't expect minSpeparation >= -b2_linearSlop because we don't
// 	// push the separation above -b2_linearSlop.
// 	return minSeparation >= -3.0f * b2_linearSlop;
// }

// // Sequential position solver for position constraints.
// bool b2ContactSolver::SolveTOIPositionConstraints(int32 toiIndexA, int32 toiIndexB)
// {
// 	float minSeparation = 0.0f;

// 	for (int32 i = 0; i < m_count; ++i)
// 	{
// 		b2ContactPositionConstraint* pc = m_positionConstraints + i;

// 		int32 indexA = pc->indexA;
// 		int32 indexB = pc->indexB;
// 		b2Vec2 localCenterA = pc->localCenterA;
// 		b2Vec2 localCenterB = pc->localCenterB;
// 		int32 pointCount = pc->pointCount;

// 		float mA = 0.0f;
// 		float iA = 0.0f;
// 		if (indexA == toiIndexA || indexA == toiIndexB)
// 		{
// 			mA = pc->invMassA;
// 			iA = pc->invIA;
// 		}

// 		float mB = 0.0f;
// 		float iB = 0.0f;
// 		if (indexB == toiIndexA || indexB == toiIndexB)
// 		{
// 			mB = pc->invMassB;
// 			iB = pc->invIB;
// 		}

// 		b2Vec2 cA = m_positions[indexA].c;
// 		float aA = m_positions[indexA].a;

// 		b2Vec2 cB = m_positions[indexB].c;
// 		float aB = m_positions[indexB].a;

// 		// Solve normal constraints
// 		for (int32 j = 0; j < pointCount; ++j)
// 		{
// 			b2Transform xfA, xfB;
// 			xfA.q.Set(aA);
// 			xfB.q.Set(aB);
// 			xfA.p = cA - b2Mul(xfA.q, localCenterA);
// 			xfB.p = cB - b2Mul(xfB.q, localCenterB);

// 			b2PositionSolverManifold psm;
// 			psm.Initialize(pc, xfA, xfB, j);
// 			b2Vec2 normal = psm.normal;

// 			b2Vec2 point = psm.point;
// 			float separation = psm.separation;

// 			b2Vec2 rA = point - cA;
// 			b2Vec2 rB = point - cB;

// 			// Track max constraint error.
// 			minSeparation = b2Min(minSeparation, separation);

// 			// Prevent large corrections and allow slop.
// 			float C = b2Clamp(b2_toiBaumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

// 			// Compute the effective mass.
// 			float rnA = b2Cross(rA, normal);
// 			float rnB = b2Cross(rB, normal);
// 			float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

// 			// Compute normal impulse
// 			float impulse = K > 0.0f ? - C / K : 0.0f;

// 			b2Vec2 P = impulse * normal;

// 			cA -= mA * P;
// 			aA -= iA * b2Cross(rA, P);

// 			cB += mB * P;
// 			aB += iB * b2Cross(rB, P);
// 		}

// 		m_positions[indexA].c = cA;
// 		m_positions[indexA].a = aA;

// 		m_positions[indexB].c = cB;
// 		m_positions[indexB].a = aB;
// 	}

// 	// We can't expect minSpeparation >= -b2_linearSlop because we don't
// 	// push the separation above -b2_linearSlop.
// 	return minSeparation >= -1.5f * b2_linearSlop;
