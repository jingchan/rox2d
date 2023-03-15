use crate::common::{Vec2, Transform};

const MAX_MANIFOLD_POINTS: usize = 2;
const MAX_POLYGON_VERTICES: usize = 8;
// const uint8 b2_nullFeature = UCHAR_MAX;

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
#[derive(Clone, Copy, Debug)]
enum ContactFeatureType {
    Vertex,
    Face,
}

#[derive(Clone, Copy, Debug)]
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
#[derive(Clone, Copy, Debug)]
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
#[derive(Clone, Copy, Debug)]
struct ManifoldPoint {
    /// Usage depends on manifold type.
    pub local_point: Vec2,
    /// The non-penetration impulse.
    pub normal_impulse: f32,
    /// The friction impulse.
    pub tangent_impulse: f32,
    /// Uniquely identifies a contact point between two shapes.
    id: ContactId,
}

#[derive(Copy, Clone, Debug, Default)]
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
#[derive(Clone, Debug)]
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
            normal: Vec2::ZERO,
            points: [Vec2::ZERO; MAX_MANIFOLD_POINTS],
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
            panic!("Manifold point count is zero.")
        }

        match manifold.manifold_type {
            ManifoldType::Circles => {
                let normal = Vec2::new(1.0, 0.0);
                let point_a = xf_a * manifold.local_point;
                let point_b = xf_b * manifold.points[0].local_point;
                if point_a.distance_squared(point_b)
                    > f32::EPSILON * f32::EPSILON
                {
                    normal = point_b - point_a;
                    normal.normalize();
                }

                let c_a = point_a + radius_a * normal;
                let c_b = point_b - radius_b * normal;
                let points = [0.5 * (c_a + c_b), 0.0];
                let separations = [Vec2::dot(c_b - c_a, normal), 0.0];
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
                        + (radius_a - (clip_point - plane_point).dot(normal))
                            * normal;
                    let c_b = clip_point - radius_b * normal;
                    points[i] = 0.5 * (c_a + c_b);
                    separations[i] = (c_b - c_a).dot(normal);
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
                        + (radius_b - (clip_point - plane_point).dot(normal))
                            * normal;
                    let c_a = clip_point - radius_a * normal;
                    points[i] = 0.5 * (c_a + c_b);
                    separations[i] = (c_a - c_b).dot(normal);
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
#[derive(Debug, Clone, Copy)]
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
#[derive(Debug, Clone, Copy)]
struct ClipVertex {
    v: Vec2,
    id: ContactId,
}

/// Ray-cast input data. The ray extends from p1 to p1 + max_fraction * (p2 -
/// p1).
#[derive(Debug, Clone, Copy)]
struct RayCastInput {
    pub p1: Vec2,
    pub p2: Vec2,
    pub max_fraction: f32,
}

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1
/// and p2 come from RayCastInput.
#[derive(Debug, Clone, Copy)]
struct RayCastOutput {
    pub normal: Vec2,
    pub fraction: f32,
}

/// An axis aligned bounding box.
#[derive(Debug, Clone, Copy, Default)]
pub struct Aabb {
    /// The lower vertex.
    pub lower_bound: Vec2,
    /// The upper vertex.
    pub upper_bound: Vec2,
}

impl Aabb {
    /// Get the center of the AABB.
    #[inline]
    pub fn center(&self) -> Vec2 {
        0.5 * (self.lower_bound + self.upper_bound)
    }

    /// Get the extents of the AABB (half-widths).
    #[inline]
    pub fn extents(&self) -> Vec2 {
        0.5 * (self.upper_bound - self.lower_bound)
    }

    /// Get the perimeter length
    pub fn get_perimeter(&self) -> f32 {
        let wx = self.upper_bound.x - self.lower_bound.x;
        let wy = self.upper_bound.y - self.lower_bound.y;
        2.0 * (wx + wy)
    }

    /// Combine this AABBs with another.
    #[inline]
    pub fn combine(&self, other: &Aabb) -> Self {
        let lower_bound = self.lower_bound.min(other.lower_bound);
        let upper_bound = self.upper_bound.min(other.upper_bound);
        Self {
            lower_bound,
            upper_bound,
        }
    }

    /// Is the given AABB contained within this AABB?
    #[inline]
    pub fn contains(&self, other: &Aabb) -> bool {
        let result = self.lower_bound.x <= other.lower_bound.x
            && self.lower_bound.y <= other.lower_bound.y
            && self.upper_bound.x >= other.upper_bound.x
            && self.upper_bound.y >= other.upper_bound.y;
        result
    }

    // From Real-time Collision Detection, p179.
    pub fn ray_cast(
        &self,
        output: &mut RayCastOutput,
        input: &RayCastInput,
    ) -> bool {
        let mut tmin = -f32::MAX;
        let mut tmax = f32::MAX;

        let p = input.p1;
        let d = input.p2 - input.p1;
        let abs_d = d.abs();

        let mut normal = Vec2::ZERO;

        for i in 0..2 {
            if abs_d.get(i) < f32::EPSILON {
                // Parallel.
                if p.get(i) < self.lower_bound.get(i)
                    || self.upper_bound.get(i) < p.get(i)
                {
                    return false;
                }
            } else {
                let inv_d = 1.0 / d.get(i);
                let mut t1 = (self.lower_bound.get(i) - p.get(i)) * inv_d;
                let mut t2 = (self.upper_bound.get(i) - p.get(i)) * inv_d;

                // Sign of the normal vector.
                let mut s = -1.0;

                if t1 > t2 {
                    core::mem::swap(&mut t1, &mut t2);
                    s = 1.0;
                }

                // Push the min up
                if t1 > tmin {
                    normal = Vec2::ZERO;
                    *normal.get_mut(i) = s;
                    tmin = t1;
                }

                // Pull the max down
                tmax = tmax.min(t2);

                if tmin > tmax {
                    return false;
                }
            }
        }

        // Does the ray start inside the box?
        // Does the ray intersect beyond the max fraction?
        if tmin < 0.0 || input.max_fraction < tmin {
            return false;
        }

        // Intersection.
        output.fraction = tmin;
        output.normal = normal;
        true
    }

    // bool b2AABB::RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const
    // {
    // 	float tmin = -b2_maxFloat;
    // 	float tmax = b2_maxFloat;

    // 	b2Vec2 p = input.p1;
    // 	b2Vec2 d = input.p2 - input.p1;
    // 	b2Vec2 absD = b2Abs(d);

    // 	b2Vec2 normal;

    // 	for (int32 i = 0; i < 2; ++i)
    // 	{
    // 		if (absD(i) < b2_epsilon)
    // 		{
    // 			// Parallel.
    // 			if (p(i) < lowerBound(i) || upperBound(i) < p(i))
    // 			{
    // 				return false;
    // 			}
    // 		}
    // 		else
    // 		{
    // 			float inv_d = 1.0f / d(i);
    // 			float t1 = (lowerBound(i) - p(i)) * inv_d;
    // 			float t2 = (upperBound(i) - p(i)) * inv_d;

    // 			// Sign of the normal vector.
    // 			float s = -1.0f;

    // 			if (t1 > t2)
    // 			{
    // 				b2Swap(t1, t2);
    // 				s = 1.0f;
    // 			}

    // 			// Push the min up
    // 			if (t1 > tmin)
    // 			{
    // 				normal.SetZero();
    // 				normal(i) = s;
    // 				tmin = t1;
    // 			}

    // 			// Pull the max down
    // 			tmax = b2Min(tmax, t2);

    // 			if (tmin > tmax)
    // 			{
    // 				return false;
    // 			}
    // 		}
    // 	}

    // 	// Does the ray start inside the box?
    // 	// Does the ray intersect beyond the max fraction?
    // 	if (tmin < 0.0f || input.maxFraction < tmin)
    // 	{
    // 		return false;
    // 	}

    // 	// Intersection.
    // 	output->fraction = tmin;
    // 	output->normal = normal;
    // 	return true;
    // }
}

/// Convex hull used for polygon collision.
#[derive(Debug, Clone, Copy)]
struct Hull {
    points: [Vec2; MAX_POLYGON_VERTICES],
    count: usize,
}
