use std::rc::Rc;

use super::{
    body::Body,
    math::{Mat2x2, Vec2},
    world::World,
};

pub enum JointType {
    Unknown,
    Revolute,
    Prismatic,
    Distance,
    Pulley,
    Mouse,
    Gear,
    Wheel,
    Weld,
    Friction,
    Rope,
    Motor,
}

pub struct Jacobian {
    linear: Vec2,
    angular1: f32,
    angular2: f32,
}

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
#[derive(Debug, Clone)]
pub struct JointEdge {
    /// Provides quick access to the other body attached.
    pub other: Rc<Body>,
    /// The joint.
    pub joint: Rc<Joint>,
    /// The previous joint edge in the body's joint list.
    pub prev: Option<Rc<JointEdge>>,
    /// The next joint edge in the body's joint list.
    pub next: Option<Rc<JointEdge>>,
}

/// Joint definitions are used to construct joints.
struct JointDef {
    /// the joint type is set automatically for concrete joint types.
    joint_type: JointType,
    /// Use this to attach application specific data to your joints.
    user_data: Option<Box<dyn std::any::Any>>,
    /// The first attached body.
    body1: Rc<Body>,
    /// The second attached body.
    body2: Rc<Body>,
    /// Set this flag to true if the attached bodies should collide.
    collide_connected: bool,
}

#[derive(Debug, Clone)]
pub struct Joint {
    pub joint_type: JointType,
    pub prev: Option<Rc<Joint>>,
    pub next: Option<Rc<Joint>>,
    pub edge_a: JointEdge,
    pub edge_b: JointEdge,
    pub body_a: Rc<Body>,
    pub body_b: Rc<Body>,

    pub index: i32,

    pub island_flag: bool,
    pub collide_connected: bool,

    pub user_data: Option<Box<dyn std::any::Any>>,
}
