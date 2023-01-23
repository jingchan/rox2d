use std::any::Any;

use super::{
    broad_phase::BroadPhase, collision::Aabb, shape::Shape, Transform,
};

const LENGTH_UNITS_PER_METER: f32 = 1.0;

pub struct FixtureDef {
    pub shape: Shape,
    pub density: f32,
    pub friction: f32,
    pub restitution: f32,
    pub restitution_threshold: f32,
    pub is_sensor: bool,
    pub filter: Filter,
    pub user_data: Option<Box<dyn Any>>,
}

impl Default for FixtureDef {
    fn default() -> Self {
        Self {
            shape: Shape::default(),
            density: 0.0,
            friction: 0.2,
            restitution: 0.0,
            restitution_threshold: 1.0 * LENGTH_UNITS_PER_METER,
            is_sensor: false,
            filter: Filter::default(),
            user_data: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Fixture {
    pub shape: Shape,
    pub density: f32,
    pub friction: f32,
    pub restitution: f32,
    pub restitution_threshold: f32,

    proxies: Vec<FixtureProxy>,
    pub(crate) proxy_count: usize,

    pub is_sensor: bool,

    pub filter: Filter,
    // pub user_data: Option<Box<dyn Any>>,
}

impl Fixture {
    pub fn synchronize(
        &mut self,
        broad_phase: &mut BroadPhase,
        transform1: &Transform,
        transform2: &Transform,
    ) {
        if self.proxy_count == 0 {
            return;
        }

        for i in 0..self.proxy_count {
            let proxy = &mut self.proxies[i];

            // Compute an AABB that covers the swept shape (may miss some rotation effect).
            let mut aabb1 = Aabb::default();
            let mut aabb2 = Aabb::default();
            self.shape
                .compute_aabb(&mut aabb1, transform1, proxy.child_index);
            self.shape
                .compute_aabb(&mut aabb2, transform2, proxy.child_index);

            proxy.aabb = aabb1.combine(&aabb2);

            let displacement = aabb2.center() - aabb1.center();

            broad_phase.move_proxy(proxy.proxy_id, &proxy.aabb, displacement);
        }
    }
}

/// This proxy is used internally to connect fixtures to the broad-phase.
#[derive(Debug, Clone)]
struct FixtureProxy {
    aabb: Aabb,
    fixture: Fixture,
    child_index: usize,
    proxy_id: usize,
}

#[derive(Debug, Clone, Copy)]
pub struct Filter {
    pub category_bits: u16,
    pub mask_bits: u16,
    pub group_index: i16,
}

impl Default for Filter {
    fn default() -> Self {
        Self {
            category_bits: 0x0001,
            mask_bits: 0xFFFF,
            group_index: 0,
        }
    }
}
