/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
// Collision
use super::settings::LENGTH_UNITS_PER_METER;

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
pub const MAX_MANIFOLD_POINTS: usize = 2;

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
pub const AABB_EXTENSION: f32 = 0.1 * LENGTH_UNITS_PER_METER;

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
pub const AABB_MULTIPLIER: f32 = 4.0;

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant. In meters.
pub const LINEAR_SLOP: f32 = 0.005 * LENGTH_UNITS_PER_METER;

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
pub const ANGULAR_SLOP: f32 = 2.0 / 180.0 * std::f32::consts::PI;

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
pub const POLYGON_RADIUS: f32 = 2.0 * LINEAR_SLOP;

/// Maximum number of sub-steps per contact in continuous physics simulation.
pub const MAX_SUB_STEPS: i32 = 8;

// Dynamics

/// Maximum number of contacts to be handled to solve a TOI impact.
pub const MAX_TOI_CONTACTS: usize = 32;

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot. Meters.
pub const MAX_LINEAR_CORRECTION: f32 = 0.2 * LENGTH_UNITS_PER_METER;

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
pub const MAX_ANGULAR_CORRECTION: f32 = 8.0 / 180.0 * std::f32::consts::PI;

/// The maximum linear translation of a body per step. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this. Meters.
pub const MAX_TRANSLATION: f32 = 2.0 * LENGTH_UNITS_PER_METER;
pub const MAX_TRANSLATION_SQUARED: f32 = MAX_TRANSLATION * MAX_TRANSLATION;

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
pub const MAX_ROTATION: f32 = 0.5 * std::f32::consts::PI;
pub const MAX_ROTATION_SQUARED: f32 = MAX_ROTATION * MAX_ROTATION;

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
pub const BAUMGARTE: f32 = 0.2;
pub const TOI_BAUMGARTE: f32 = 0.75;

// Sleep

/// The time that a body must be still before it will go to sleep.
pub const TIME_TO_SLEEP: f32 = 0.5;

/// A body cannot sleep if its linear velocity is above this tolerance.
pub const LINEAR_SLEEP_TOLERANCE: f32 = 0.01 * LENGTH_UNITS_PER_METER;

/// A body cannot sleep if its angular velocity is above this tolerance.
pub const ANGULAR_SLEEP_TOLERANCE: f32 = 2.0 / 180.0 * std::f32::consts::PI;

// /// Dump to a file. Only one dump file allowed at a time.
// void b2OpenDump(const char* fileName);
// void b2Dump(const char* string, ...);
// void b2CloseDump();

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
pub struct Version {
    /// significant changes
    pub major: i32,
    /// incremental changes
    pub minor: i32,
    /// bug fixes
    pub revision: i32,
}

/// Current version.
pub const VERSION: Version = Version {
    major: 2,
    minor: 4,
    revision: 0,
};
