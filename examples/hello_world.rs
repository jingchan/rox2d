#![feature(get_many_mut)]
// use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use ::rox2d::{contact_manager::ContactManager, world::World};
use rox2d::*;

fn main() {
    let gravity = Vec2::new(0.0, -10.0);
    let contact_manager = ContactManager::new();
    let mut world = World::new(gravity, contact_manager);

    create_ground(&mut world);
    create_body(&mut world);
}

fn create_ground(world: &mut World) {
    // Define the ground body.
    let mut ground_body_def = BodyDef::new();
    ground_body_def.position = Vec2::new(0.0, -10.0);

    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    let ground_body = world.create_body(&ground_body_def);
    // TODO: Unwind the world creation functions.
    // let ground_body = Body::new(&ground_body_def, &world);

    // Define the ground box shape.
    let mut ground_box = PolygonShape::new();
    // The extents are the half-widths of the box.
    ground_box.set_as_box(50.0, 10.0);

    // Add the ground fixture to the ground body.
    let mut ground_fixture_def = FixtureDef::new();
    ground_fixture_def.shape = Some(&ground_box);
    ground_fixture_def.density = 0.0;
    ground_fixture_def.friction = 0.3;
    ground_body.create_fixture(&ground_fixture_def);
}

fn create_body(world: &mut World) {
    // Define the dynamic body. We set its position and call the body factory.
    let mut body_def = BodyDef::new();
    body_def.body_type = BodyType::Dynamic;
    body_def.position = Vec2::new(0.0, 4.0);
    let body = world.create_body(&body_def);

    // Define another box shape for our dynamic body.
    let mut dynamic_box = PolygonShape::new();
    dynamic_box.set_as_box(1.0, 1.0);

    // Define the dynamic body fixture.
    let mut fixture_def = FixtureDef::new();
    fixture_def.shape = Some(&dynamic_box);

    // Set the box density to be non-zero, so it will be dynamic.
    fixture_def.density = 1.0;

    // Override the default friction.
    fixture_def.friction = 0.3;

    // Add the shape to the body.
    body.create_fixture(&fixture_def);
}
