use std::{cmp::Ordering, rc::Rc};

use super::math::Vec2;

#[derive(Debug, Clone)]
pub struct Body {
    pub id: usize,

    pub position: Vec2,
    pub rotation: f32,

    pub velocity: Vec2,
    pub angular_velocity: f32,

    pub force: Vec2,
    pub torque: f32,

    pub size: Vec2,

    pub friction: f32,
    pub mass: f32,
    pub inv_mass: f32,

    /// Moment of inertia (I)
    ///
    /// The rotational inertia of a body is the tendency of that body to resist
    /// changes in its rotational motion. This is the rotational equivalent of
    /// mass.
    moment_of_inertia: f32,

    /// Inverse moment of inertia (I^-1)
    pub inv_moment_of_inertia: f32,
}

impl Body {
    #[inline(always)]
    pub fn new(size: Vec2, mass: f32) -> Self {
        let moment_of_inertia = if mass.is_finite() {
            mass * size.dot(size) / 12.0
        } else {
            f32::INFINITY
        };

        Self {
            size,
            mass,
            inv_mass: mass.recip(),
            moment_of_inertia,
            inv_moment_of_inertia: moment_of_inertia.recip(),
            ..Default::default()
        }
    }
}

impl Default for Body {
    #[inline(always)]
    fn default() -> Self {
        Self {
            id: 0,

            position: Vec2::ZERO,
            rotation: 0.0,

            velocity: Vec2::ZERO,
            angular_velocity: 0.0,

            force: Vec2::ZERO,
            torque: 0.0,

            size: Vec2::ONE,

            friction: 0.2,
            mass: f32::INFINITY,
            inv_mass: 0.0,
            moment_of_inertia: f32::INFINITY,
            inv_moment_of_inertia: 0.0,
        }
    }
}

impl PartialEq for Body {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl PartialOrd for Body {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.id.cmp(&other.id))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let rec = f32::INFINITY.recip();
        assert_eq!(rec, 0.0);
    }
}
