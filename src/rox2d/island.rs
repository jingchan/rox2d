use super::{
    common::{
        MAX_ROTATION, MAX_ROTATION_SQUARED, MAX_TRANSLATION,
        MAX_TRANSLATION_SQUARED,
    },
    contact::Contact,
    contact_solver::{ContactSolver, ContactSolverDef},
    joint::Joint,
    time_step::{SolverData, TimeStep},
    Body, BodyType, Vec2,
};

pub struct Position {
    pub c: Vec2,
    pub a: f32,
}

pub struct Velocity {
    pub v: Vec2,
    pub w: f32,
}

struct IntermediateData {
    pub pos: Position,
    pub vel: Velocity,
}

struct Island {
    // allocator: &'static mut dyn Allocator,
    // listener: Option<&'static dyn ContactListener>,
    bodies: Vec<Body>,
    contacts: Vec<Contact>,
    joints: Vec<Joint>,

    // Internal ephemeral data
    data: Vec<IntermediateData>,
}

impl Island {
    fn new(
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

    fn clear(&mut self) {
        self.bodies.clear();
        self.contacts.clear();
        self.joints.clear();
    }

    fn solve(&self, step: TimeStep, gravity: Vec2) {
        let h = step.dt;
        for (body, data) in self.bodies.iter_mut().zip(self.data.iter_mut()) {
            let c = body.sweep.c;
            let a = body.sweep.a;
            let v = body.linear_velocity;
            let w = body.angular_velocity;

            body.sweep.c0 = body.sweep.c;
            body.sweep.a0 = body.sweep.a;

            if body.ty == BodyType::DYNAMIC {
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
        // 	contactSolver.StoreImpulses();
        // 	profile->solveVelocity = timer.GetMilliseconds();

        // 	// Integrate positions
        // 	for (int32 i = 0; i < m_bodyCount; ++i)
        // 	{
        // 		b2Vec2 c = m_positions[i].c;
        // 		float a = m_positions[i].a;
        // 		b2Vec2 v = m_velocities[i].v;
        // 		float w = m_velocities[i].w;

        // 		// Check for large velocities
        // 		b2Vec2 translation = h * v;
        // 		if (b2Dot(translation, translation) > b2_maxTranslationSquared)
        // 		{
        // 			float ratio = b2_maxTranslation / translation.Length();
        // 			v *= ratio;
        // 		}

        // 		float rotation = h * w;
        // 		if (rotation * rotation > b2_maxRotationSquared)
        // 		{
        // 			float ratio = b2_maxRotation / b2Abs(rotation);
        // 			w *= ratio;
        // 		}

        // 		// Integrate
        // 		c += h * v;
        // 		a += h * w;

        // 		m_positions[i].c = c;
        // 		m_positions[i].a = a;
        // 		m_velocities[i].v = v;
        // 		m_velocities[i].w = w;
        // 	}

        // 	// Solve position constraints
        // 	timer.Reset();
        // 	bool positionSolved = false;
        // 	for (int32 i = 0; i < step.positionIterations; ++i)
        // 	{
        // 		bool contactsOkay = contactSolver.SolvePositionConstraints();

        // 		bool jointsOkay = true;
        // 		for (int32 j = 0; j < m_jointCount; ++j)
        // 		{
        // 			bool jointOkay = m_joints[j]->SolvePositionConstraints(solverData);
        // 			jointsOkay = jointsOkay && jointOkay;
        // 		}

        // 		if (contactsOkay && jointsOkay)
        // 		{
        // 			// Exit early if the position errors are small.
        // 			positionSolved = true;
        // 			break;
        // 		}
        // 	}

        // 	// Copy state buffers back to the bodies
        // 	for (int32 i = 0; i < m_bodyCount; ++i)
        // 	{
        // 		b2Body* body = m_bodies[i];
        // 		body->m_sweep.c = m_positions[i].c;
        // 		body->m_sweep.a = m_positions[i].a;
        // 		body->m_linearVelocity = m_velocities[i].v;
        // 		body->m_angularVelocity = m_velocities[i].w;
        // 		body->SynchronizeTransform();
        // 	}

        // 	profile->solvePosition = timer.GetMilliseconds();

        // 	Report(contactSolver.m_velocityConstraints);

        // 	if (allowSleep)
        // 	{
        // 		float minSleepTime = b2_maxFloat;

        // 		const float linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
        // 		const float angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

        // 		for (int32 i = 0; i < m_bodyCount; ++i)
        // 		{
        // 			b2Body* b = m_bodies[i];
        // 			if (b->GetType() == b2_staticBody)
        // 			{
        // 				continue;
        // 			}

        // 			if ((b->m_flags & b2Body::e_autoSleepFlag) == 0 ||
        // 				b->m_angularVelocity * b->m_angularVelocity > angTolSqr ||
        // 				b2Dot(b->m_linearVelocity, b->m_linearVelocity) > linTolSqr)
        // 			{
        // 				b->m_sleepTime = 0.0f;
        // 				minSleepTime = 0.0f;
        // 			}
        // 			else
        // 			{
        // 				b->m_sleepTime += h;
        // 				minSleepTime = b2Min(minSleepTime, b->m_sleepTime);
        // 			}
        // 		}

        // 		if (minSleepTime >= b2_timeToSleep && positionSolved)
        // 		{
        // 			for (int32 i = 0; i < m_bodyCount; ++i)
        // 			{
        // 				b2Body* b = m_bodies[i];
        // 				b->SetAwake(false);
        // 			}
        // 		}
        // 	}
    }
}
