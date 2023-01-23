use super::{
    body::Body, contact::ContactFlags, joint::Joint, time_step::TimeStep,
    BodyFlags, BodyType, Vec2,
};

#[derive(Debug, Clone)]
pub struct World {
    // block_allocator: BlockAllocator,
    // stack_allocator: StackAllocator,
    pub contact_manager: ContactManager,
    bodies: Vec<Body>,
    joints: Vec<Joint>,

    gravity: Vec2,
    allow_sleep: bool,

    // destruction_listener: Option<DestructionListener>,
    // debug_draw: Option<DebugDraw>,

    // This is used to compute the time step ratio to
    // support a variable time step.
    inv_dt0: f32,

    new_contacts: bool,
    locked: bool,
    clear_forces: bool,

    // These are for debugging the solver.
    warm_starting: bool,
    continuous_physics: bool,
    sub_stepping: bool,

    step_complete: bool,
    // profile: Profile,
}

impl World {
    pub const ACCUMULATE_IMPULSES: bool = true;
    pub const WARM_STARTING: bool = true;
    pub const POSITION_CORRECTION: bool = true;

    pub fn new(gravity: Vec2, contact_manager: ContactManager) -> Self {
        Self {
            contact_manager: ContactManager::new(),
            bodies: Vec::new(),
            joints: Vec::new(),
            gravity,
            allow_sleep: true,
            inv_dt0: 0.0,
            new_contacts: false,
            locked: false,
            clear_forces: true,
            warm_starting: true,
            continuous_physics: true,
            sub_stepping: false,
            step_complete: true,
        }
    }

    pub fn solve(&mut self, step: &TimeStep) {
        // let timer = Timer::new();

        // Size the island for the worst case.
        let mut island = Island::new(
            self.bodies.len(),
            self.joints.len(),
            // &mut self.stack_allocator,
            &mut self.contact_manager.contact_listener,
        );

        // Clear all the island flags.
        for body in &mut self.bodies {
            body.flags &= !BodyFlags::ISLAND;
        }
        for contact in &mut self.contact_manager.contacts {
            contact.flags &= !ContactFlags::ISLAND;
        }
        for joint in &mut self.joints {
            joint.island_flag = false;
        }

        // Build and simulate all awake islands.
        let mut stack_size = self.bodies.len();
        let mut stack = self.stack_allocator.allocate(stack_size);
        for seed in &mut self.bodies {
            if seed.flags & BodyFlags::ISLAND != BodyFlags::empty() {
                continue;
            }

            if !seed.is_awake() || !seed.is_active() {
                continue;
            }

            // The seed can be dynamic or kinematic.
            if seed.body_type == BodyType::Static {
                continue;
            }

            // Reset island and stack.
            island.clear();
            let mut stack_count = 0;
            stack[stack_count] = seed;
            stack_count += 1;
            seed.flags |= BodyFlags::ISLAND;

            // Perform a depth first search (DFS) on the constraint graph.
            while stack_count > 0 {
                // Grab the next body off the stack and add it to the island.
                let b = stack[stack_count - 1];
                stack_count -= 1;
                island.add(b);

                // Make sure the body is awake (without resetting sleep timer).
                b.flags |= BodyFlags::AWAKE;

                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if b.body_type == BodyType::Static {
                    continue;
                }

                // Search all contacts connected to this body.
                for ce in &b.contact_edges {
                    let contact = ce.contact;

                    // Has this contact already been added to an island?
                    if contact.flags & ContactFlags::ISLAND != 0 {
                        continue;
                    }

                    // Is this contact solid and touching?
                    if !contact.is_enabled() || !contact.is_touching() {
                        continue;
                    }

                    // Skip sensors.
                    let sensor_a = contact.fixture_a.is_sensor();
                    let sensor_b = contact.fixture_b.is_sensor();
                    if sensor_a || sensor_b {
                        continue;
                    }

                    island.add(contact);
                    contact.flags |= ContactFlags::ISLAND;

                    let other = ce.other;

                    // Was the other body already added to this island?
                    if other.flags & BodyFlags::ISLAND != 0 {
                        continue;
                    }

                    assert!(stack_count < stack_size);
                    stack[stack_count] = other;
                    stack_count += 1;
                    other.flags |= BodyFlags::ISLAND;

                    // Search all joints connect to this body.
                    for je in &b.joint_edges {
                        if je.joint.island_flag {
                            continue;
                        }

                        let other = je.other;

                        // Don't simulate joints connected to inactive bodies.
                        if other.is_enabled() == false {
                            continue;
                        }

                        island.add(je.joint);
                        je.joint.island_flag = true;

                        if other.flags & BodyFlags::ISLAND != 0 {
                            continue;
                        }

                        assert!(stack_count < stack_size);
                        stack[stack_count] = other;
                        stack_count += 1;
                        other.flags |= BodyFlags::ISLAND;
                    }
                }

                island.solve(step, &self.gravity, self.allow_sleep);

                // Post solve cleanup.
                for i in 0..island.body_count {
                    // Allow static bodies to participate in other islands.
                    let b = island.bodies[i];
                    if b.body_type == BodyType::Static {
                        b.flags &= !BodyFlags::ISLAND;
                    }
                }
            }
        }

        // self.stack_allocator.free(stack, stack_size);

        // let timer = timer::Timer::new();
        // Synchronize fixtures, check for out of range bodies.
        for b in &mut self.bodies {
            // If a body was not in an island then it did not move.
            if b.flags & BodyFlags::ISLAND == BodyFlags::empty() {
                continue;
            }

            if b.body_type == BodyType::Static {
                continue;
            }

            // Update fixtures (for broad-phase).
            b.synchronize_fixtures();
        }

        // Look for new contacts.
        self.contact_manager.find_new_contacts();
        // profile.broadphase = timer.elapsed();
    }
}
