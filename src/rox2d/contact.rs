use std::rc::Rc;

use bitflags::bitflags;

use super::{collision::Manifold, fixture::Fixture, Body};

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
    flags: u32,
    prev: Option<Rc<Contact>>,
    next: Option<Rc<Contact>>,

    node_a: ContactEdge,
    node_b: ContactEdge,

    pub fixture_a: Fixture,
    pub fixture_b: Fixture,

    index_a: i32,
    index_b: i32,

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
}
