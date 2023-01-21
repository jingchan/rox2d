use super::Vec2;

/// This characterizes how forces get applied.
struct MassData {
    /// The mass, in kilograms.
    mass: f32,

    /// The centroid relative to the origin.
    center: Vec2,

    /// The rotational inertia of the shape.
    /// TODO: Do we need to say "about the local origin".
    inertia: f32,
}

enum ShapeType {
    Circle,
    Edge,
    Polygon,
    Chain,
}

pub struct Shape {
    pub shape_type: ShapeType,
    pub radius: f32,
    pub mass_data: MassData,
}
