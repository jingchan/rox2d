use bitflags::bitflags;
use std::cmp::Ordering;

use crate::{Sweep, Transform};

use super::{contact::ContactEdge, fixture::Fixture, math::Vec2, World};

/// The body type.
/// - static: zero mass, zero velocity, may be manually moved
/// - kinematic: zero mass, non-zero velocity set by user, moved by solver
/// - dynamic: positive mass,non-zero velocity determined by forces, moved by
/// solver
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
    struct BodyFlags: u16 {
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

    pub world: World,
    pub prev: Option<Body>,
    pub next: Option<Body>,

    pub fixture_list: Option<Fixture>,
    pub fixture_count: i32,

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

    pub user_data: Option<Box<dyn std::any::Any>>,
}

impl Body {
    #[inline(always)]
    pub fn new(def: &BodyDef, world: &World) -> Self {
        let mut body = Body {
            body_type: def.body_type,
            flags: BodyFlags::ACTIVE,
            island_index: 0,
            xf: Transform::new(def.position, def.angle),
            sweep: Sweep::new(),
            linear_velocity: def.linear_velocity,
            angular_velocity: def.angular_velocity,
            force: Vec2::ZERO,
            torque: 0.0,
            world: world.clone(),
            prev: None,
            next: None,
            fixture_list: None,
            fixture_count: 0,
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
            user_data: def.user_data.clone(),
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

        body.sweep.local_center.set_zero();
        body.sweep.c0 = body.xf.p;
        body.sweep.c = body.xf.p;
        body.sweep.a0 = body.xf.q.get_angle();
        body.sweep.a = body.xf.q.get_angle();
        body.sweep.alpha0 = 0.0;

        body
    }
    // 	b2Assert(bd->position.IsValid());
    // 	b2Assert(bd->linearVelocity.IsValid());
    // 	b2Assert(b2IsValid(bd->angle));
    // 	b2Assert(b2IsValid(bd->angularVelocity));
    // 	b2Assert(b2IsValid(bd->angularDamping) && bd->angularDamping >= 0.0f);
    // 	b2Assert(b2IsValid(bd->linearDamping) && bd->linearDamping >= 0.0f);

    // 	m_flags = 0;

    // 	if (bd->bullet)
    // 	{
    // 		m_flags |= e_bulletFlag;
    // 	}
    // 	if (bd->fixedRotation)
    // 	{
    // 		m_flags |= e_fixedRotationFlag;
    // 	}
    // 	if (bd->allowSleep)
    // 	{
    // 		m_flags |= e_autoSleepFlag;
    // 	}
    // 	if (bd->awake && bd->type != b2_staticBody)
    // 	{
    // 		m_flags |= e_awakeFlag;
    // 	}
    // 	if (bd->enabled)
    // 	{
    // 		m_flags |= e_enabledFlag;
    // 	}

    // 	m_world = world;

    // 	m_xf.p = bd->position;
    // 	m_xf.q.Set(bd->angle);

    // 	m_sweep.localCenter.SetZero();
    // 	m_sweep.c0 = m_xf.p;
    // 	m_sweep.c = m_xf.p;
    // 	m_sweep.a0 = bd->angle;
    // 	m_sweep.a = bd->angle;
    // 	m_sweep.alpha0 = 0.0f;

    // 	m_jointList = nullptr;
    // 	m_contactList = nullptr;
    // 	m_prev = nullptr;
    // 	m_next = nullptr;

    // 	m_linearVelocity = bd->linearVelocity;
    // 	m_angularVelocity = bd->angularVelocity;

    // 	m_linearDamping = bd->linearDamping;
    // 	m_angularDamping = bd->angularDamping;
    // 	m_gravityScale = bd->gravityScale;

    // 	m_force.SetZero();
    // 	m_torque = 0.0f;

    // 	m_sleepTime = 0.0f;

    // 	m_type = bd->type;

    // 	m_mass = 0.0f;
    // 	m_invMass = 0.0f;

    // 	m_I = 0.0f;
    // 	m_invI = 0.0f;

    // 	m_userData = bd->userData;

    // 	m_fixtureList = nullptr;
    // 	m_fixtureCount = 0;
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
