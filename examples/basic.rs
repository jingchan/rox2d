#![feature(get_many_mut)]
use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use rox2d::rox2d::*;

#[derive(Debug, Resource)]
struct World {
    inner: rox2d::World,
}
#[derive(Debug, Component)]
struct Body {
    body_id: usize,
}

const NUM_ITERATIONS: usize = 5;

impl Default for World {
    fn default() -> Self {
        let world = rox2d::World::new(rox2d::Vec2::new(0.0, -9.81));

        Self { inner: world }
    }
}

fn main() {
    App::new()
        .init_resource::<World>()
        .add_plugins(DefaultPlugins)
        .add_startup_system(setup)
        .add_system(update)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut world: ResMut<World>,
) {
    commands.spawn(Camera2dBundle::default());

    // Rectangle
    commands.spawn(SpriteBundle {
        sprite: Sprite {
            color: Color::rgb(0.25, 0.25, 0.75),
            custom_size: Some(bevy::prelude::Vec2::new(50.0, 100.0)),
            ..default()
        },
        ..default()
    });

    // Circle
    commands.spawn(MaterialMesh2dBundle {
        mesh: meshes.add(shape::Circle::new(50.).into()).into(),
        material: materials.add(ColorMaterial::from(Color::PURPLE)),
        transform: Transform::from_translation(Vec3::new(-100., 0., 0.)),
        ..default()
    });

    let body_id = world
        .inner
        .add_body(rox2d::Body::new(rox2d::Vec2::new(20.0, 20.0), 100.0));
    // Hexagon
    commands.spawn((
        MaterialMesh2dBundle {
            mesh: meshes.add(shape::RegularPolygon::new(50., 6).into()).into(),
            material: materials.add(ColorMaterial::from(Color::TURQUOISE)),
            transform: Transform::from_translation(Vec3::new(100., 0., 0.)),
            ..default()
        },
        Body { body_id },
    ));
}

fn update(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &Body)>,
    mut world: ResMut<World>,
) {
    world.inner.step(time.delta_seconds(), NUM_ITERATIONS);

    for (mut transform, body) in &mut query {
        let body = world.inner.get_body(body.body_id).unwrap();
        transform.translation =
            Vec3::new(body.position.x * 100.0, body.position.y * 100.0, 0.0);
    }

    // world.Step(timeStep);

    // for (int i = 0; i < numBodies; ++i)
    // 	DrawBody(bodies + i);

    // for (int i = 0; i < numJoints; ++i)
    // 	DrawJoint(joints + i);

    // glPointSize(4.0f);
    // glColor3f(1.0f, 0.0f, 0.0f);
    // glBegin(GL_POINTS);
    // std::map<ArbiterKey, Arbiter>::const_iterator iter;
    // for (iter = world.arbiters.begin(); iter != world.arbiters.end(); ++iter)
    // {
    // 	const Arbiter& arbiter = iter->second;
    // 	for (int i = 0; i < arbiter.numContacts; ++i)
    // 	{
    // 		Vec2 p = arbiter.contacts[i].position;
    // 		glVertex2f(p.x, p.y);
    // 	}
    // }
}
