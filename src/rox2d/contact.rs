use std::rc::Rc;

use super::{collision::Manifold, fixture::Fixture, Body};

#[derive(Debug, Clone)]
pub struct ContactEdge {
    other: Rc<Body>,
    contact: Rc<Contact>,
    prev: Option<Rc<ContactEdge>>,
    next: Option<Rc<ContactEdge>>,
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
