use super::{
    body::Body,
    fixture::Fixture,
    math::{Mat2x2, Vec2},
};

const MAX_MANIFOLD_POINTS: usize = 2;
const RELATIVE_TOLERANCE: f32 = 0.95;
const ABSOLUTE_TOLERANCE: f32 = 0.01;
const MAX_POLYGON_VERTICES: usize = 8;

/// Contact ids to facilitate warm starting.
ContactId
{
	cb2ContactFeature cf;
	uint32 key;					///< Used to quickly compare contact ids.
};
enum ManifoldType {
    Circles,
    FaceA,
    FaceB,
}

pub struct Manifold {
    /// The points of contact.
    points: [ManifoldPoint; 2],
    /// Not use for type::e_points.
    local_normal: Vec2,
    /// Usage depends on manifold type.j
    local_point: Vec2,
    manifold_type: ManifoldType,
    /// The number of manifold points.
    point_count: i32,
}

struct ManifoldPoint {
    /// Usage depends on manifold type.
    local_point: Vec2,
    /// The non-penetration impulse.
    normal_impulse: f32,
    /// The friction impulse.
    tangent_impulse: f32,
    /// Uniquely identifies a contact point between two shapes.
    id: ContactID,
}

struct WorldManifold {
    /// World vector pointing from A to B.
    normal: Vec2,
    /// World contact point (point of intersection).
    points: [Vec2; MAX_MANIFOLD_POINTS],
    /// A negative value indicates overlap, in meters.
    separations: [f32; MAX_MANIFOLD_POINTS],
}
enum ContactFeatureType {
    Vertex,
    Face,
}

struct ContactFeature {
    index_a: u8,
    index_b: u8,
    type_a: ContactFeatureType,
    type_b: ContactFeatureType,
}

/// This is used for determining the state of contact points.
enum PointState {
    /// Point does not exist.
    NullState,
    /// Point was added in the update.
    AddState,
    /// Point persisted across the update.
    PersistState,
    /// Point was removed in the update.
    RemoveState,
}

/// Used for computing contact manifolds.
struct ClipVertex {
    v: Vec2,
    id: ContactId,
}

/// Ray-cast input data. The ray extends from p1 to p1 + max_fraction * (p2 -
/// p1).
struct RayCastInput {
    pub p1: Vec2,
    pub p2: Vec2,
    pub max_fraction: f32,
}

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1
/// and p2 come from RayCastInput.
struct RayCastOutput {
    pub normal: Vec2,
    pub fraction: f32,
}

/// An axis aligned bounding box.
struct Aabb {
    /// The lower vertex.
    pub lower_bound: Vec2,
    /// The upper vertex.
    pub upper_bound: Vec2,
}

/// Convex hull used for polygon collision.
struct Hull {
    points: [Vec2; MAX_POLYGON_VERTICES],
    count: usize,
}
