use std::default;

use crate::common::Transform;

use super::{collision::Aabb};

/// This characterizes how forces get applied.
#[derive(Debug, Default, Clone, Copy)]
struct MassData {
    /// The mass, in kilograms.
    mass: f32,

    /// The centroid relative to the origin.
    center: Vec2,

    /// The rotational inertia of the shape.
    /// TODO: Do we need to say "about the local origin".
    inertia: f32,
}

#[derive(Debug, Clone, Copy, Default)]
enum ShapeType {
    #[default]
    Circle,
    Edge,
    Polygon,
    Chain,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Shape {
    pub shape_type: ShapeType,
    pub radius: f32,
    pub mass_data: MassData,
    pub count: usize,
}

impl Shape {
    pub fn compute_aabb(&self, xf: &Transform, child_index: usize) -> Aabb {
        match self.shape_type {
            ShapeType::Circle => {
                let circle = self as *const Shape as *const CircleShape;
                let circle = unsafe { &*circle };
                circle.compute_aabb(xf, child_index)
            }
            ShapeType::Edge => {
                let edge = self as *const Shape as *const EdgeShape;
                let edge = unsafe { &*edge };
                edge.compute_aabb(xf, child_index)
            }
            ShapeType::Polygon => {
                let polygon = self as *const Shape as *const PolygonShape;
                let polygon = unsafe { &*polygon };
                polygon.compute_aabb(xf, child_index)
            }
            ShapeType::Chain => {
                let chain = self as *const Shape as *const ChainShape;
                let chain = unsafe { &*chain };
                chain.compute_aabb(xf, child_index)
            }
        }
    }

    pub fn child_count(&self) -> usize {
        match self.shape_type {
            ShapeType::Circle => 1,
            ShapeType::Edge => 1,
            ShapeType::Polygon => 1,
            ShapeType::Chain => self.count - 1,
        }
    }
}

/// A circle shape.
#[derive(Debug, Clone, Copy, Default)]
pub struct CircleShape {
    pub shape: Shape,
    pub position: Vec2,
}

impl CircleShape {
    pub fn compute_aabb(&self, xf: &Transform, child_index: usize) -> Aabb {
        let p = xf * self.position;
        Aabb {
            lower_bound: p - Vec2::new(self.shape.radius, self.shape.radius),
            upper_bound: p + Vec2::new(self.shape.radius, self.shape.radius),
        }
    }
}

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
///
/// This structure is stored across time steps, so we keep it small.
/// Note: the edge supports smooth collision, however the chain does not.
///
/// TODO: we should be able to compute the normal using a cross product.
/// TODO: should be able to compute the normal using a cross product.
#[derive(Debug, Clone, Copy, Default)]
pub struct EdgeShape {
    pub shape: Shape,
    pub vertex1: Vec2,
    pub vertex2: Vec2,
    pub vertex0: Vec2,
    pub vertex3: Vec2,
    pub has_vertex0: bool,
    pub has_vertex3: bool,
}

impl EdgeShape {
    pub fn compute_aabb(&self, xf: &Transform, child_index: usize) -> Aabb {
        let v1 = *xf * self.vertex1;
        let v2 = *xf * self.vertex2;

        Aabb {
            lower_bound: v1.min(v2),
            upper_bound: v1.max(v2),
        }
    }
}

/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
///
/// In most cases you should not need many vertices for a convex polygon.
pub struct PolygonShape {
    pub shape: Shape,
    pub vertices: Vec<Vec2>,
    pub normals: Vec<Vec2>,
}

impl PolygonShape {
    pub fn compute_aabb(&self, xf: &Transform, child_index: usize) -> Aabb {
        let lower = xf * self.vertices[0];
        let upper = lower;

        for i in 1..self.vertices.len() {
            let v = xf * self.vertices[i];
            lower = lower.min(v);
            upper = upper.max(v);
        }

        let r = Vec2::new(self.shape.radius, self.shape.radius);
        Aabb {
            lower_bound: lower - r,
            upper_bound: upper + r,
        }
    }
}

/// A chain shape is a free form sequence of line segments.
/// The chain has one-sided collision, with the surface normal pointing to the
/// right of the edge.
/// This provides a counter-clockwise winding like the polygon shape.
/// Connectivity information is used to create smooth collisions.
/// WARNING: The chain will not collide properly if there are self-intersections.
pub struct ChainShape {
    pub shape: Shape,
    pub vertices: Vec<Vec2>,
    pub has_prev_vertex: bool,
    pub has_next_vertex: bool,
    pub prev_vertex: Vec2,
    pub next_vertex: Vec2,
}

impl ChainShape {
    pub fn compute_aabb(&self, xf: &Transform, child_index: usize) -> Aabb {
        let lower = xf * self.vertices[0];
        let upper = lower;

        for i in 1..self.vertices.len() {
            let v = xf * self.vertices[i];
            lower = lower.min(v);
            upper = upper.max(v);
        }

        let r = Vec2::new(self.shape.radius, self.shape.radius);

        Aabb {
            lower_bound: lower - r,
            upper_bound: upper + r,
        }
    }
}
