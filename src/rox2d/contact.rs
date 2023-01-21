use super::{collision::Manifold, fixture::Fixture, Body};

pub struct ContactEdge {
    other: Body,
    contact: Contact,
    prev: Option<Box<ContactEdge>>,
    next: Option<Box<ContactEdge>>,
}

#[derive(Debug, Default, Clone)]
pub struct Contact {
    flags: u32,
    prev: Option<Box<Contact>>,
    next: Option<Box<Contact>>,

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
    pub fn get_maniforld(&self) -> &Manifold {
        &self.manifold
    }
}
