use crate::Transform;

use super::Vec2;

const MAX_MANIFOLD_POINTS: usize = 2;
const MAX_POLYGON_VERTICES: usize = 8;
// const uint8 b2_nullFeature = UCHAR_MAX;

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
#[derive(Clone, Copy)]
enum ContactFeatureType {
    Vertex,
    Face,
}

#[derive(Clone, Copy)]
struct ContactFeature {
    /// Feature index on shapeA
    index_a: u8,
    /// Feature index on shapeB
    index_b: u8,
    /// The feature type on shapeA
    type_a: ContactFeatureType,
    /// The feature type on shapeB
    type_b: ContactFeatureType,
}

/// Contact ids to facilitate warm starting.
#[derive(Clone, Copy)]
enum ContactId {
    /// Used to quickly compare contact ids.
    Key(u32),
    /// Contact ids to facilitate warm starting.
    Feature(ContactFeature),
}

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
#[derive(Copy, Clone)]
struct ManifoldPoint {
    /// Usage depends on manifold type.
    pub local_point: Vec2,
    /// The non-penetration impulse.
    normal_impulse: f32,
    /// The friction impulse.
    tangent_impulse: f32,
    /// Uniquely identifies a contact point between two shapes.
    id: ContactId,
}

#[derive(Copy, Clone, Default)]
pub enum ManifoldType {
    #[default]
    Circles,
    FaceA,
    FaceB,
}

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
pub struct Manifold {
    /// The points of contact.
    pub points: [ManifoldPoint; 2],
    /// Not use for type::e_points.
    local_normal: Vec2,
    /// Usage depends on manifold type.j
    local_point: Vec2,
    pub manifold_type: ManifoldType,
    /// The number of manifold points.
    pub point_count: i32,
}

/// This is used to compute the current state of a contact manifold.
pub struct WorldManifold {
    /// World vector pointing from A to B.
    pub normal: Vec2,
    /// World contact point (point of intersection).
    pub points: [Vec2; MAX_MANIFOLD_POINTS],
    /// A negative value indicates overlap, in meters.
    pub separations: [f32; MAX_MANIFOLD_POINTS],
}

impl WorldManifold {
    pub fn new() -> Self {
        Self {
            normal: Vec2::zero(),
            points: [Vec2::zero(); MAX_MANIFOLD_POINTS],
            separations: [0.0; MAX_MANIFOLD_POINTS],
        }
    }

    pub fn initialize(
        manifold: &Manifold,
        xf_a: &Transform,
        radius_a: f32,
        xf_b: &Transform,
        radius_b: f32,
    ) -> Self {
        if manifold.point_count == 0 {
            return;
        }

        match manifold.manifold_type {
            ManifoldType::Circles => {
                let normal = Vec2::new(1.0, 0.0);
                let point_a = Transform::mul_vec2(xf_a, manifold.local_point);
                let point_b =
                    Transform::mul_vec2(xf_b, manifold.points[0].local_point);
                if Vec2::distance_squared(point_a, point_b)
                    > f32::EPSILON * f32::EPSILON
                {
                    normal = point_b - point_a;
                    normal.normalize();
                }

                let c_a = point_a + radius_a * normal;
                let c_b = point_b - radius_b * normal;
                let points = [0.5 * (c_a + c_b)];
                let separations = [Vec2::dot(c_b - c_a, normal)];
                Self {
                    normal,
                    points,
                    separations,
                }
            }
            ManifoldType::FaceA => {
                let normal =
                    Transform::mul_rot_vec2(xf_a.q, manifold.local_normal);
                let plane_point =
                    Transform::mul_vec2(xf_a, manifold.local_point);

                let points = [Vec2::zero(); MAX_MANIFOLD_POINTS];
                let separations = [0.0; MAX_MANIFOLD_POINTS];
                for i in 0..manifold.point_count {
                    let clip_point = Transform::mul_vec2(
                        xf_b,
                        manifold.points[i].local_point,
                    );
                    let c_a = clip_point
                        + (radius_a
                            - Vec2::dot(clip_point - plane_point, normal))
                            * normal;
                    let c_b = clip_point - radius_b * normal;
                    points[i] = 0.5 * (c_a + c_b);
                    separations[i] = Vec2::dot(c_b - c_a, normal);
                }
                Self {
                    normal,
                    points,
                    separations,
                }
            }
            ManifoldType::FaceB => {
                let normal =
                    Transform::mul_rot_vec2(xf_b.q, manifold.local_normal);
                let plane_point =
                    Transform::mul_vec2(xf_b, manifold.local_point);

                let points = [Vec2::zero(); MAX_MANIFOLD_POINTS];
                let separations = [0.0; MAX_MANIFOLD_POINTS];
                for i in 0..manifold.point_count {
                    let clip_point = Transform::mul_vec2(
                        xf_a,
                        manifold.points[i].local_point,
                    );
                    let c_b = clip_point
                        + (radius_b
                            - Vec2::dot(clip_point - plane_point, normal))
                            * normal;
                    let c_a = clip_point - radius_a * normal;
                    points[i] = 0.5 * (c_a + c_b);
                    separations[i] = Vec2::dot(c_a - c_b, normal);
                }

                // Ensure normal points from A to B.
                normal = -normal;
                Self {
                    normal,
                    points,
                    separations,
                }
            }
        }
    }
}

/// This is used for determining the state of contact points.
enum PointState {
    /// point does not exist
    NullState,
    /// point was added in the update
    AddState,
    /// point persisted across the update
    PersistState,
    /// point was removed in the update
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
pub struct Aabb {
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
