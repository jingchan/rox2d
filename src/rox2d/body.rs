use bitflags::bitflags;

use crate::{
    broad_phase::BroadPhase,
    common::{Rot, Sweep, Transform, Vec2},
    fixture::FixtureDef,
    joint::JointEdge,
};

use super::{contact::ContactEdge, fixture::Fixture, world::World};

/// The body type.
/// - static: zero mass, zero velocity, may be manually moved
/// - kinematic: zero mass, non-zero velocity set by user, moved by solver
/// - dynamic: positive mass,non-zero velocity determined by forces, moved by
/// solver
#[derive(Debug, Clone, PartialEq)]
pub enum BodyType {
    Static,
    Kinematic,
    Dynamic,
}

pub struct BodyDef {
    /// The body type: static, kinematic, or dynamic.
    /// Note: if a dynamic body would have zero mass, the mass is set to one.
    pub body_type: BodyType,
    /// The world position of the body. Avoid creating bodies at the origin
    /// since this can lead to many overlapping shapes.
    pub position: Vec2,
    /// The world angle of the body in radians.
    pub angle: f32,
    /// The linear velocity of the body's origin in world co-ordinates.
    pub linear_velocity: Vec2,
    /// The angular velocity of the body.
    pub angular_velocity: f32,
    /// Linear damping is use to reduce the linear velocity. The damping parameter
    /// can be larger than 1.0 but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    /// Units are 1/time
    pub linear_damping: f32,
    /// Angular damping is use to reduce the angular velocity. The damping
    /// parameter can be larger than 1.0 but the damping effect becomes sensitive
    /// to the time step when the damping parameter is large.
    /// Units are 1/time
    pub angular_damping: f32,
    /// Set this flag to false if this body should never fall asleep. Note that
    /// this increases CPU usage.
    pub allow_sleep: bool,
    /// Is this body initially sleeping?
    pub awake: bool,
    /// Should this body be prevented from rotating? Useful for characters.
    pub fixed_rotation: bool,
    /// Is this a fast moving body that should be prevented from tunneling through
    /// other moving bodies? Note that all bodies are prevented from tunneling
    /// through kinematic and static bodies. This setting is only considered on
    /// dynamic bodies.
    /// @warning You should use this flag sparingly since it increases processing time.
    pub bullet: bool,
    /// Does this body start out active?
    pub active: bool,
    /// Use this to store application specific body data.
    pub user_data: Option<Box<dyn std::any::Any>>,
    /// Scale the gravity applied to this body.
    pub gravity_scale: f32,
}

impl BodyDef {
    pub fn new() -> BodyDef {
        BodyDef {
            body_type: BodyType::Static,
            position: Vec2::ZERO,
            angle: 0.0,
            linear_velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            allow_sleep: true,
            awake: true,
            fixed_rotation: false,
            bullet: false,
            active: true,
            user_data: None,
            gravity_scale: 1.0,
        }
    }
}

bitflags! {
    pub struct BodyFlags: u16 {
        const ISLAND = 0x0001;
        const AWAKE = 0x0002;
        const AUTOSLEEP = 0x0004;
        const BULLET = 0x0008;
        const FIXED_ROTATION = 0x0010;
        const ACTIVE = 0x0020;
        const TOI = 0x0040;
    }
}

/// A rigid body. These are created via `World::create_body`.
#[derive(Debug, Clone)]
pub struct Body {
    pub body_type: BodyType,
    pub flags: BodyFlags,

    pub island_index: i32,

    pub xf: Transform,
    pub sweep: Sweep,

    pub linear_velocity: Vec2,
    pub angular_velocity: f32,

    pub force: Vec2,
    pub torque: f32,

    // pub world: World,
    pub prev: Option<Box<Body>>,
    pub next: Option<Box<Body>>,

    pub fixtures: Vec<Fixture>,
    // pub fixture_list: Option<Fixture>,
    // pub fixture_count: i32,
    pub joint_list: Option<JointEdge>,
    pub contact_list: Option<ContactEdge>,

    pub mass: f32,
    pub inv_mass: f32,

    /// Rotational inertia about the center of mass.
    pub inertia: f32,
    pub inv_inertia: f32,

    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,

    pub sleep_time: f32,
    // pub user_data: Option<Box<dyn std::any::Any>>,
}

impl Body {
    #[inline(always)]
    // pub fn new(def: &BodyDef, world: &World) -> Self {
    pub fn new(def: &BodyDef) -> Self {
        let mut body = Body {
            body_type: def.body_type,
            flags: BodyFlags::ACTIVE,
            island_index: 0,
            xf: Transform::new(def.position, def.angle),
            sweep: Sweep::default(),
            linear_velocity: def.linear_velocity,
            angular_velocity: def.angular_velocity,
            force: Vec2::ZERO,
            torque: 0.0,
            // Notes: Circular reference here to world, we should get rid of
            // this, but try to see if it's used anywhere.
            // world: world.clone(),
            prev: None,
            next: None,
            fixtures: Vec::new(),
            // fixture_list: None,
            // fixture_count: 0,
            joint_list: None,
            contact_list: None,
            mass: 0.0,
            inv_mass: 0.0,
            inertia: 0.0,
            inv_inertia: 0.0,
            linear_damping: def.linear_damping,
            angular_damping: def.angular_damping,
            gravity_scale: def.gravity_scale,
            sleep_time: 0.0,
            // user_data: def.user_data.clone(),
        };

        if def.awake {
            body.flags |= BodyFlags::AWAKE;
        }

        if def.allow_sleep {
            body.flags |= BodyFlags::AUTOSLEEP;
        }

        if def.fixed_rotation {
            body.flags |= BodyFlags::FIXED_ROTATION;
        }

        if def.bullet {
            body.flags |= BodyFlags::BULLET;
        }

        if def.active {
            body.flags |= BodyFlags::ACTIVE;
        }

        body.sweep.local_center = Vec2::ZERO;
        body.sweep.c0 = body.xf.p;
        body.sweep.c = body.xf.p;
        body.sweep.a0 = body.xf.q.get_angle();
        body.sweep.a = body.xf.q.get_angle();
        body.sweep.alpha0 = 0.0;

        body
    }

    pub fn is_awake(&self) -> bool {
        (self.flags & BodyFlags::AWAKE) == BodyFlags::AWAKE
    }

    pub fn is_active(&self) -> bool {
        (self.flags & BodyFlags::ACTIVE) == BodyFlags::ACTIVE
    }

    pub fn is_fixed_rotation(&self) -> bool {
        (self.flags & BodyFlags::FIXED_ROTATION) == BodyFlags::FIXED_ROTATION
    }

    pub fn synchronize_fixtures(&mut self, broad_phase: &mut BroadPhase) {
        if self.is_awake() {
            let xf1 = Transform::new(
                self.sweep.c0
                    - Rot::new(self.sweep.a0) * self.sweep.local_center,
                self.sweep.a0,
            );
            for fixture in self.fixture_list.iter_mut() {
                fixture.synchronize(broad_phase, &xf1, &self.xf);
            }
        } else {
            for fixture in self.fixture_list.iter_mut() {
                fixture.synchronize(broad_phase, &self.xf, &self.xf);
            }
        }
    }

    pub fn create_fixture(
        &mut self,
        world: &World,
        def: &FixtureDef,
    ) -> Option<&Fixture> {
        assert!(world.is_locked() == false);
        if world.is_locked() {
            return None;
        }

        // let mut allocator = &world.block_allocator;

        // let memory = allocator.allocate(std::mem::size_of::<Fixture>());
        // let fixture = unsafe { std::mem::transmute::<*mut u8, &mut Fixture>(memory) };
        // fixture.create(allocator, self, def);
        let fixture = Fixture::new(def);

        if self.is_active() {
            let broad_phase = &mut world.contact_manager.broad_phase;
            fixture.create_proxies(broad_phase, &self.xf);
        }

        self.fixtures.push(fixture);
    }

    // This is used to prevent connected bodies from colliding.
    // It may lie, depending on the collideConnected flag.
    pub fn should_collide(&self, other: &Body) -> bool {
        // At least one body should be dynamic.
        if self.body_type != BodyType::Dynamic
            && other.body_type != BodyType::Dynamic
        {
            return false;
        }

        // Does a joint prevent collision?
        for joint_edge in self.joint_list.iter() {
            if joint_edge.other == other
                && joint_edge.joint.collide_connected == false
            {
                return false;
            }
        }

        true
    }

    /// TODO: Should we call this set_transform?
    pub fn synchronize_transform(&self) {
        self.xf.q.set_angle(self.sweep.a);
        self.xf.p =
            self.sweep.c - Rot::new(self.sweep.a) * self.sweep.local_center;
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
