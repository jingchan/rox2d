#![feature(get_many_mut)]

pub mod common;

#[cfg(not(feature = "use-mini"))]
pub mod rox2d;

#[cfg(feature = "use-mini")]
pub mod rox2d_mini;

pub use common::*;
// pub use rox2d_mini::body::*;
// pub use rox2d_mini::math::*;
// pub use rox2d_mini::world::*;
