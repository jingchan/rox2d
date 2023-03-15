use std::any::Any;

use crate::common::Transform;

use super::{broad_phase::BroadPhase, collision::Aabb, shape::Shape};

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

    pub proxies: Vec<FixtureProxy>,
    // pub(crate) proxy_count: usize,
    pub is_sensor: bool,

    pub filter: Filter,
    // pub user_data: Option<Box<dyn Any>>,
}

impl Fixture {
    pub fn new(def: &FixtureDef) -> Self {
        Self {
            shape: def.shape.clone(),
            density: def.density,
            friction: def.friction,
            restitution: def.restitution,
            restitution_threshold: def.restitution_threshold,
            proxies: Vec::new(),
            is_sensor: def.is_sensor,
            filter: def.filter.clone(),
            // user_data: def.user_data.clone(),
        }
    }

    //
    pub fn synchronize(
        &mut self,
        broad_phase: &mut BroadPhase,
        transform1: &Transform,
        transform2: &Transform,
    ) {
        for proxy in self.proxies.iter_mut() {
            // Compute an AABB that covers the swept shape (may miss some
            // rotation effect).
            let mut aabb1 = Aabb::default();
            let mut aabb2 = Aabb::default();
            let aabb1 = self.shape.compute_aabb(transform1, proxy.child_index);
            let aabb2 = self.shape.compute_aabb(transform2, proxy.child_index);

            proxy.aabb = aabb1.combine(&aabb2);

            let displacement = aabb2.center() - aabb1.center();

            // Notes:
            // - This seems to be sweeping the combined size of the AABB this
            // seems it would cover a lot of unnecessary area.
            // - The displacement is passed down into the broad phase, so it
            // seems this logic can be delegated (or even deferred).
            broad_phase.move_proxy(proxy.proxy_id, &proxy.aabb, displacement);
        }
    }

    // This creates AABBs in the broad-phase for this fixture.
    pub fn create_proxies(
        &mut self,
        broad_phase: &mut BroadPhase,
        xf: &Transform,
    ) {
        // Create proxies in the broad-phase.
        let proxy_count = self.shape.child_count();

        for child_index in 0..proxy_count {
            let aabb = self.shape.compute_aabb(xf, child_index);
            let proxy_id = broad_phase.create_proxy(&aabb);
            let proxy = FixtureProxy {
                aabb,
                fixture: self.clone(),
                child_index: child_index,
                proxy_id,
            };
            self.proxies.push(proxy);
        }
    }

    pub fn is_sensor(&self) -> bool {
        self.is_sensor
    }
}

/// This proxy is used internally to connect fixtures to the broad-phase.
#[derive(Debug, Clone)]
struct FixtureProxy {
    pub aabb: Aabb,
    pub fixture: Fixture,
    pub child_index: usize,
    pub proxy_id: usize,
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
