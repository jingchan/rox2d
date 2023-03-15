use std::rc::Rc;

use bitflags::bitflags;

use crate::body::Body;

use super::{collision::Manifold, fixture::Fixture};

#[derive(Debug, Clone)]
pub struct ContactEdge {
    other: Rc<Body>,
    contact: Rc<Contact>,
    prev: Option<Rc<ContactEdge>>,
    next: Option<Rc<ContactEdge>>,
}

bitflags! {
    pub struct ContactFlags: u32 {
        const ISLAND     = 0x0001;
        const TOUCHING   = 0x0002;
        const ENABLED    = 0x0004;
        const FILTER     = 0x0008;
        const BULLET_HIT = 0x0010;
        const TOI        = 0x0020;
    }
}

#[derive(Debug, Clone)]
pub struct Contact {
    pub flags: ContactFlags,
    // prev: Option<Rc<Contact>>,
    // next: Option<Rc<Contact>>,
    node_a: ContactEdge,
    node_b: ContactEdge,

    pub fixture_a: Fixture,
    pub fixture_b: Fixture,

    pub child_index_a: usize,
    pub child_index_b: usize,

    manifold: Manifold,

    toi_count: i32,
    toi: f32,

    pub friction: f32,
    pub restitution: f32,
    pub restitution_threshold: f32,

    pub tangent_speed: f32,
}

impl Contact {
    /// Get the contact manifold. Do not modify the manifold unless you understand the
    /// internals of Box2D.
    pub fn get_manifold(&self) -> &Manifold {
        &self.manifold
    }

    // Update the contact manifold and touching status.
    // Note: do not assume the fixture AABBs are overlapping or are valid.
    pub fn update(&mut self) {
        let old_manifold = self.manifold.clone();

        // Re-enable this contact.
        self.flags |= ContactFlags::ENABLED;

        let mut touching = false;
        let was_touching = self.flags.contains(ContactFlags::TOUCHING);

        let sensor_a = self.fixture_a.is_sensor();
        let sensor_b = self.fixture_b.is_sensor();
        let sensor = sensor_a || sensor_b;

        let body_a = self.fixture_a.body.clone();
        let body_b = self.fixture_b.body.clone();
        let xf_a = body_a.get_transform();
        let xf_b = body_b.get_transform();

        // Is this contact a sensor?
        if sensor {
            let shape_a = self.fixture_a.shape.clone();
            let shape_b = self.fixture_b.shape.clone();
            let index_a = self.child_index_a;
            let index_b = self.child_index_b;

            touching =
                shape_a.test_overlap(index_a, &shape_b, index_b, &xf_a, &xf_b);

            // Sensors don't generate manifolds.
            self.manifold.point_count = 0;
        } else {
            // evaluate
            // Evaluate(&m_manifold, xfA, xfB);

            touching = self.manifold.point_count > 0;

            // Match old contact ids to new contact ids and copy the
            // stored impulses to warm start the solver.
            for i in 0..self.manifold.point_count {
                let mut mp2 = &mut self.manifold.points[i];
                mp2.normal_impulse = 0.0;
                mp2.tangent_impulse = 0.0;
                let id2 = mp2.id;

                for j in 0..old_manifold.point_count {
                    let mp1 = &old_manifold.points[j];

                    if mp1.id.key == id2.key {
                        mp2.normal_impulse = mp1.normal_impulse;
                        mp2.tangent_impulse = mp1.tangent_impulse;
                        break;
                    }
                }
            }

            if touching != was_touching {
                body_a.set_awake(true);
                body_b.set_awake(true);
            }
        }

        if touching {
            self.flags.insert(ContactFlags::TOUCHING);
        } else {
            self.flags.remove(ContactFlags::TOUCHING);
        }

        // TODO: Add listener hooks here.
        // if was_touching == false && touching == true && sensor == false {
        // }
    }
}
