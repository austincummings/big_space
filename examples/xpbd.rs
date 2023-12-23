use bevy::{math::DVec3, prelude::*};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_xpbd_3d::{
    components::{Collider, LinearDamping, LinearVelocity, LockedAxes, RigidBody},
    plugins::{
        setup::{Physics, PhysicsTime},
        sync::SyncConfig,
        PhysicsDebugPlugin, PhysicsPlugins, SyncPlugin,
    },
    resources::Gravity,
};
use big_space::bevy_xpbd::floating_origin_sync::FloatingOriginSyncPlugin;
use big_space::{debug::FloatingOriginDebugPlugin, FloatingOrigin, FloatingOriginPlugin, GridCell};

fn main() {
    let physics_schedule = PostUpdate;

    App::new()
        .add_plugins((
            DefaultPlugins.build().disable::<TransformPlugin>(),
            FloatingOriginPlugin::<i8>::new(5.0, 0.0),
            FloatingOriginDebugPlugin::<i8>::default(),
            PhysicsPlugins::new(physics_schedule.clone())
                .build()
                .disable::<SyncPlugin>()
                .add(FloatingOriginSyncPlugin::<i8>::new(
                    physics_schedule.clone(),
                )),
            PhysicsDebugPlugin::default(),
            WorldInspectorPlugin::default(),
        ))
        .insert_resource(ForceScale(0.05))
        .insert_resource(Gravity(DVec3::ZERO))
        .insert_resource(SyncConfig {
            position_to_transform: true,
            transform_to_position: true,
        })
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                control,
                draw_origin_gizmo,
                draw_floating_origin_marker_gizmo,
                draw_non_floating_origin_marker_gizmo,
            ),
        )
        .run();
}

#[derive(Resource, Deref, DerefMut)]
struct ForceScale(f32);

#[derive(Component)]
struct Controlled1;

#[derive(Component)]
struct Controlled2;

fn setup(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 0.0, 40.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    commands.spawn((
        Name::new("Floating Origin Cube"),
        SpatialBundle {
            transform: Transform::from_xyz(-2.0, 0.0, 0.0),
            ..default()
        },
        GridCell::<i8>::default(),
        RigidBody::default(),
        Collider::default(),
        LinearDamping::default(),
        LockedAxes::new()
            .lock_translation_z()
            .lock_rotation_y()
            .lock_rotation_x()
            .lock_rotation_z(),
        FloatingOrigin,
        Controlled1,
    ));

    commands.spawn((
        Name::new("Cube"),
        SpatialBundle {
            transform: Transform::from_xyz(2.0, 0.0, 0.0),
            ..default()
        },
        GridCell::<i8>::default(),
        RigidBody::default(),
        Collider::default(),
        LinearDamping::default(),
        LockedAxes::new()
            .lock_translation_z()
            .lock_rotation_y()
            .lock_rotation_x()
            .lock_rotation_z(),
        Controlled2,
    ));
}

fn control(
    keyboard_input: Res<Input<KeyCode>>,
    force_scale: Res<ForceScale>,
    mut controlled1: Query<
        (&mut LinearVelocity, &mut LinearDamping),
        (With<Controlled1>, Without<Controlled2>),
    >,
    mut controlled2: Query<
        (&mut LinearVelocity, &mut LinearDamping),
        (With<Controlled2>, Without<Controlled1>),
    >,
) {
    let mut force1 = Vec3::ZERO;
    let mut force2 = Vec3::ZERO;

    if keyboard_input.pressed(KeyCode::W) {
        force1 += Vec3::Y * force_scale.0;
    }

    if keyboard_input.pressed(KeyCode::Up) {
        force2 += Vec3::Y * force_scale.0;
    }

    if keyboard_input.pressed(KeyCode::S) {
        force1 -= Vec3::Y * force_scale.0;
    }

    if keyboard_input.pressed(KeyCode::Down) {
        force2 -= Vec3::Y * force_scale.0;
    }

    if keyboard_input.pressed(KeyCode::A) {
        force1 -= Vec3::X * force_scale.0;
    }

    if keyboard_input.pressed(KeyCode::Left) {
        force2 -= Vec3::X * force_scale.0;
    }

    if keyboard_input.pressed(KeyCode::D) {
        force1 += Vec3::X * force_scale.0;
    }

    if keyboard_input.pressed(KeyCode::Right) {
        force2 += Vec3::X * force_scale.0;
    }

    for (mut linear_velocity, _) in controlled1.iter_mut() {
        linear_velocity.0 += force1.as_dvec3();
    }

    for (mut linear_velocity, _) in controlled2.iter_mut() {
        linear_velocity.0 += force2.as_dvec3();
    }

    let linear_damping_value = if keyboard_input.pressed(KeyCode::Space) {
        10.0
    } else {
        0.0
    };

    for (_, mut linear_damping) in controlled1.iter_mut() {
        linear_damping.0 = linear_damping_value;
    }

    for (_, mut linear_damping) in controlled2.iter_mut() {
        linear_damping.0 = linear_damping_value;
    }
}

fn draw_origin_gizmo(mut gizmos: Gizmos) {
    gizmos.line(Vec3::ZERO, Vec3::Y, Color::GREEN);
    gizmos.line(Vec3::ZERO, Vec3::X, Color::RED);
    gizmos.line(Vec3::ZERO, Vec3::Z, Color::BLUE);
}

fn draw_floating_origin_marker_gizmo(
    mut gizmos: Gizmos,
    query: Query<&GlobalTransform, With<FloatingOrigin>>,
) {
    for global_transform in query.iter() {
        let transform = global_transform.compute_transform();
        gizmos.sphere(
            transform.translation,
            transform.rotation,
            0.5,
            Color::PURPLE,
        );
    }
}

fn draw_non_floating_origin_marker_gizmo(
    mut gizmos: Gizmos,
    query: Query<&GlobalTransform, Without<FloatingOrigin>>,
) {
    for global_transform in query.iter() {
        let transform = global_transform.compute_transform();
        gizmos.sphere(transform.translation, transform.rotation, 0.5, Color::BLUE);
    }
}
