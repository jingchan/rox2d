#![feature(get_many_mut)]
// use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use rox2d::rox2d::*;

fn main() {
    let mut world = World::new();
    let mut dispatcher = DispatcherBuilder::new()
        .with_system(HelloWorldSystem, "hello_world", &[])
        .build();
    dispatcher.setup(&mut world);
    dispatcher.run(&mut world);
}
