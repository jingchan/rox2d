use std::any::Any;

use super::shape::Shape;



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


#[derive(Debug, Default, Clone)]
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

    pub user_data: Option<Box<dyn Any>>,
}

/// This proxy is used internally to connect fixtures to the broad-phase.
struct FixtureProxy
{
	aabb: Aabb,
	fixture: Fixture,
	child_index: usize,
	proxy_id: usize,
};

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
