#![feature(get_many_mut)]

#[cfg(not(feature = "use-mini"))]
pub mod rox2d;

#[cfg(feature = "use-mini")]
pub mod rox2d_mini;
