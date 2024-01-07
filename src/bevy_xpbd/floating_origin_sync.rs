//! Responsible for synchronizing physics components with other data, like keeping [`Position`]
//! and [`Rotation`] in sync with `Transform`.
//!
//! See [`FloatingOriginSyncPlugin`].

use std::marker::PhantomData;

use bevy::{
    ecs::{query::Has, schedule::ScheduleLabel},
    prelude::*,
    utils::intern::Interned,
};
use bevy_xpbd_3d::{
    math::{AdjustPrecision, AsF32, Scalar, Vector},
    plugins::sync::PreviousGlobalTransform,
    prelude::*,
    SubstepSchedule, SubstepSet,
};

use crate::{
    precision::GridPrecision,
    propagation::{self, propagate_transforms},
    recenter_transform_on_grid, sync_simple_transforms, update_global_from_grid, FloatingOrigin,
    FloatingOriginSettings, GridCell,
};

/// Responsible for synchronizing physics components with other data, like keeping [`Position`]
/// and [`Rotation`] in sync with `Transform`.
///
/// ## Syncing between [`Position`]/[`Rotation`] and [`Transform`]
///
/// By default, each body's `Transform` will be updated when [`Position`] or [`Rotation`]
/// change, and vice versa. This means that you can use any of these components to move
/// or position bodies, and the changes be reflected in the other components.
///
/// You can configure what data is synchronized and how it is synchronized
/// using the [`SyncConfig`] resource.
///
/// ## `Transform` hierarchies
///
/// When synchronizing changes in [`Position`] or [`Rotation`] to `Transform`,
/// the engine treats nested [rigid bodies](RigidBody) as a flat structure. This means that
/// the bodies move independently of the parents, and moving the parent will not affect the child.
///
/// If you would like a child entity to be rigidly attached to its parent, you could use a [`FixedJoint`]
/// or write your own system to handle hierarchies differently.
pub struct FloatingOriginSyncPlugin<P: GridPrecision> {
    schedule: Interned<dyn ScheduleLabel>,
    phantom: PhantomData<P>,
}

impl<P: GridPrecision> FloatingOriginSyncPlugin<P> {
    /// Creates a [`FloatingOriginSyncPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
            phantom: PhantomData,
        }
    }
}

impl<P: GridPrecision> Default for FloatingOriginSyncPlugin<P> {
    fn default() -> Self {
        Self::new(PostUpdate)
    }
}

impl<P: GridPrecision> Plugin for FloatingOriginSyncPlugin<P> {
    fn build(&self, app: &mut App) {
        app.init_resource::<SyncConfig>()
            .register_type::<SyncConfig>();

        // Initialize `PreviousGlobalTransform` and apply `Transform` changes that happened
        // between the end of the previous physics frame and the start of this physics frame.
        app.add_systems(
            self.schedule,
            ((
                sync_simple_transforms::<P>,
                propagate_transforms::<P>,
                init_previous_global_transform,
                transform_to_position,
                // Update `PreviousGlobalTransform` for the physics step's `GlobalTransform` change detection
                update_previous_global_transforms,
                recenter_positions::<P>,
            )
                .chain()
                .after(PhysicsSet::Prepare)
                .before(PhysicsSet::StepSimulation),)
                .chain()
                .run_if(|config: Res<SyncConfig>| config.transform_to_position),
        );

        // Apply `Transform`, `Position` and `Rotation` changes that happened during the physics frame.
        app.add_systems(
            self.schedule,
            (
                // Apply `Position` and `Rotation` changes to `Transform`
                position_to_transform::<P>
                    .run_if(|config: Res<SyncConfig>| config.position_to_transform),
                (
                    // Update `PreviousGlobalTransform` for next frame's `GlobalTransform` change detection
                    sync_simple_transforms::<P>,
                    propagate_transforms::<P>,
                    update_previous_global_transforms,
                )
                    .chain()
                    .run_if(|config: Res<SyncConfig>| config.transform_to_position),
                update_collider_scale,
            )
                .chain()
                .in_set(PhysicsSet::Sync),
        );

        // Update child colliders before narrow phase in substepping loop
        let substep_schedule = app
            .get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first");
        substep_schedule.add_systems(
            (
                propagate_collider_transforms::<P>,
                update_child_collider_position,
            )
                .chain()
                .after(SubstepSet::Integrate)
                .before(SubstepSet::NarrowPhase),
        );
    }
}

/// Configures what physics data is synchronized by the [`FloatingOriginSyncPlugin`] and how.
#[derive(Resource, Reflect, Clone, Debug, PartialEq, Eq)]
#[reflect(Resource)]
pub struct SyncConfig {
    /// Updates transforms based on [`Position`] and [`Rotation`] changes. Defaults to true.
    pub position_to_transform: bool,
    /// Updates [`Position`] and [`Rotation`] based on transform changes,
    /// allowing you to move bodies using `Transform`. Defaults to true.
    pub transform_to_position: bool,
}

impl Default for SyncConfig {
    fn default() -> Self {
        SyncConfig {
            position_to_transform: true,
            transform_to_position: true,
        }
    }
}

type PhysicsObjectAddedFilter = Or<(Added<RigidBody>, Added<Collider>)>;

fn init_previous_global_transform(
    mut commands: Commands,
    query: Query<(Entity, &GlobalTransform), PhysicsObjectAddedFilter>,
) {
    for (entity, transform) in &query {
        commands
            .entity(entity)
            .insert(PreviousGlobalTransform(*transform));
    }
}

#[allow(clippy::type_complexity)]
pub(crate) fn update_child_collider_position(
    mut colliders: Query<
        (
            &ColliderTransform,
            &mut Position,
            &mut Rotation,
            &ColliderParent,
        ),
        Without<RigidBody>,
    >,
    parents: Query<(&Position, &Rotation), (With<RigidBody>, With<Children>)>,
) {
    for (collider_transform, mut position, mut rotation, parent) in &mut colliders {
        let Ok((parent_pos, parent_rot)) = parents.get(parent.get()) else {
            continue;
        };

        position.0 = parent_pos.0 + parent_rot.rotate(collider_transform.translation);
        {
            *rotation = (parent_rot.0 * collider_transform.rotation.0)
                .normalize()
                .into();
        }
    }
}

#[allow(clippy::type_complexity)]
pub(crate) fn update_collider_scale(
    mut colliders: ParamSet<(
        // Root bodies
        Query<(&Transform, &mut Collider), Without<Parent>>,
        // Child colliders
        Query<(&ColliderTransform, &mut Collider), With<Parent>>,
    )>,
) {
    // Update collider scale for root bodies
    for (transform, mut collider) in &mut colliders.p0() {
        let scale = transform.scale.adjust_precision();
        if scale != collider.scale() {
            // TODO: Support configurable subdivision count for shapes that
            //       can't be represented without approximations after scaling.
            collider.set_scale(scale, 10);
        }
    }

    // Update collider scale for child colliders
    for (collider_transform, mut collider) in &mut colliders.p1() {
        if collider_transform.scale != collider.scale() {
            // TODO: Support configurable subdivision count for shapes that
            //       can't be represented without approximations after scaling.
            collider.set_scale(collider_transform.scale, 10);
        }
    }
}

/// Updates [`ColliderTransform`]s based on entity hierarchies. Each transform is computed by recursively
/// traversing the children of each rigid body and adding their transforms together to form
/// the total transform relative to the body.
///
/// This is largely a clone of `propagate_transforms` in `bevy_transform`.
#[allow(clippy::type_complexity)]
pub(crate) fn propagate_collider_transforms<P: GridPrecision>(
    origin_moved: Query<(), (Changed<GridCell<P>>, With<FloatingOrigin>)>,
    mut root_query: Query<
        (
            Entity,
            &Children,
            Ref<Transform>,
            &mut ColliderTransform,
            Option<Ref<GridCell<P>>>,
        ),
        Without<Parent>,
    >,
    transform_query: Query<
        (Ref<Transform>, &mut ColliderTransform, Option<&Children>),
        With<Parent>,
    >,
    parent_query: Query<(Entity, Ref<Parent>)>,
) {
    let origin_cell_changed = !origin_moved.is_empty();

    for (entity, children, transform, mut collider_transform, cell) in root_query.iter_mut() {
        let cell_changed = cell.as_ref().filter(|cell| cell.is_changed()).is_some();
        let transform_changed = transform.is_changed();

        if transform_changed && cell.is_none() {
            *collider_transform = ColliderTransform::from(*transform);
        }

        let changed = transform_changed || cell_changed || origin_cell_changed;

        for (child, actual_parent) in parent_query.iter_many(children) {
            assert_eq!(
                actual_parent.get(), entity,
                "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
            );
            // SAFETY:
            // - `child` must have consistent parentage, or the above assertion would panic.
            // Since `child` is parented to a root entity, the entire hierarchy leading to it is consistent.
            // - We may operate as if all descendants are consistent, since `propagate_recursive` will panic before
            //   continuing to propagate if it encounters an entity with inconsistent parentage.
            // - Since each root entity is unique and the hierarchy is consistent and forest-like,
            //   other root entities' `propagate_recursive` calls will not conflict with this one.
            // - Since this is the only place where `transform_query` gets used, there will be no conflicting fetches elsewhere.
            unsafe {
                propagate_collider_transforms_recursive(
                    &collider_transform,
                    &transform_query,
                    &parent_query,
                    child,
                    changed || actual_parent.is_changed(),
                );
            }
        }
    }
}

/// Recursively computes the [`ColliderTransform`] for `entity` and all of its descendants
/// by propagating transforms.
///
/// This is largely a clone of `propagate_recursive` in `bevy_transform`.
///
/// # Panics
///
/// If `entity`'s descendants have a malformed hierarchy, this function will panic occur before propagating
/// the transforms of any malformed entities and their descendants.
///
/// # Safety
///
/// - While this function is running, `transform_query` must not have any fetches for `entity`,
/// nor any of its descendants.
/// - The caller must ensure that the hierarchy leading to `entity`
/// is well-formed and must remain as a tree or a forest. Each entity must have at most one parent.
#[allow(clippy::type_complexity)]
unsafe fn propagate_collider_transforms_recursive(
    parent: &ColliderTransform,
    transform_query: &Query<
        (Ref<Transform>, &mut ColliderTransform, Option<&Children>),
        With<Parent>,
    >,
    parent_query: &Query<(Entity, Ref<Parent>)>,
    entity: Entity,
    mut changed: bool,
) {
    let (global_matrix, children) = {
        let Ok((transform, mut collider_transform, children)) =
            // SAFETY: This call cannot create aliased mutable references.
            //   - The top level iteration parallelizes on the roots of the hierarchy.
            //   - The caller ensures that each child has one and only one unique parent throughout the entire
            //     hierarchy.
            //
            // For example, consider the following malformed hierarchy:
            //
            //     A
            //   /   \
            //  B     C
            //   \   /
            //     D
            //
            // D has two parents, B and C. If the propagation passes through C, but the Parent component on D points to B,
            // the above check will panic as the origin parent does match the recorded parent.
            //
            // Also consider the following case, where A and B are roots:
            //
            //  A       B
            //   \     /
            //    C   D
            //     \ /
            //      E
            //
            // Even if these A and B start two separate tasks running in parallel, one of them will panic before attempting
            // to mutably access E.
            (unsafe { transform_query.get_unchecked(entity) }) else {
                return;
            };

        changed |= transform.is_changed();
        if changed {
            *collider_transform = {
                let translation = parent.transform_point(transform.translation.as_dvec3());
                let rotation = parent.rotation.0 * transform.rotation.adjust_precision();
                let scale = parent.scale * transform.scale.adjust_precision();
                ColliderTransform {
                    translation,
                    rotation: Rotation(rotation),
                    scale,
                }
            };
        }
        (*collider_transform, children)
    };

    let Some(children) = children else { return };
    for (child, actual_parent) in parent_query.iter_many(children) {
        assert_eq!(
            actual_parent.get(), entity,
            "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
        );
        // SAFETY: The caller guarantees that `transform_query` will not be fetched
        // for any descendants of `entity`, so it is safe to call `propagate_recursive` for each child.
        //
        // The above assertion ensures that each child has one and only one unique parent throughout the
        // entire hierarchy.
        unsafe {
            propagate_collider_transforms_recursive(
                &global_matrix,
                transform_query,
                parent_query,
                child,
                changed || actual_parent.is_changed(),
            );
        }
    }
}
/// Copies `GlobalTransform` changes to [`Position`] and [`Rotation`].
/// This allows users to use transforms for moving and positioning bodies and colliders.
///
/// To account for hierarchies, transform propagation should be run before this system.
#[allow(clippy::type_complexity)]
fn transform_to_position(
    mut query: Query<(
        &GlobalTransform,
        &PreviousGlobalTransform,
        &mut Position,
        Option<&AccumulatedTranslation>,
        &mut Rotation,
        &PreviousRotation,
        &CenterOfMass,
    )>,
) {
    for (
        global_transform,
        previous_transform,
        mut position,
        accumulated_translation,
        mut rotation,
        previous_rotation,
        center_of_mass,
    ) in &mut query
    {
        // Skip entity if the global transform value hasn't changed
        if *global_transform == previous_transform.0 {
            continue;
        }

        let transform = global_transform.compute_transform();
        let previous_transform = previous_transform.compute_transform();

        let pos = position.0
            + accumulated_translation.map_or(Vector::ZERO, |t| {
                get_pos_translation(t, previous_rotation, &rotation, center_of_mass)
            });

        {
            // position.0 = (previous_transform.translation
            //     + (transform.translation - previous_transform.translation))
            //     .adjust_precision()
            //     + (pos - previous_transform.translation.adjust_precision());
            position.0 = previous_transform.translation.adjust_precision()
                + (pos - previous_transform.translation.adjust_precision());
        }

        {
            // rotation.0 = (previous_transform.rotation
            //     + (transform.rotation - previous_transform.rotation)
            //     + (rotation.as_f32() - previous_transform.rotation))
            //     .normalize()
            //     .adjust_precision();
        }
    }
}

/// Compute the `Position` relative to the floating origin's cell.
fn recenter_positions<P: GridPrecision>(
    settings: Res<FloatingOriginSettings>,
    mut origin: Query<
        (
            &Transform,
            Ref<GridCell<P>>,
            &mut Position,
            &mut PreviousPosition,
        ),
        With<FloatingOrigin>,
    >,
    mut entities: Query<
        (
            &Transform,
            Ref<GridCell<P>>,
            &mut Position,
            &mut PreviousPosition,
        ),
        Without<FloatingOrigin>,
    >,
) {
    let Ok((origin_transform, origin_grid_cell, mut origin_position, mut origin_previous_position)) =
        origin.get_single_mut()
    else {
        warn!("No floating origin found, skipping position recentering");
        return;
    };

    if origin_grid_cell.is_changed() {
        // Recenter origin position
        origin_position.0 = origin_transform.translation.as_dvec3();
        origin_previous_position.0 = origin_position.0;
    }

    // Recenter entity positions relative to origin
    entities.par_iter_mut().for_each(
        |(transform, grid_cell, mut position, mut previous_position)| {
            if origin_grid_cell.is_changed() || grid_cell.is_changed() {
                let grid_cell_delta = *grid_cell - *origin_grid_cell;
                position.0 = settings.grid_position_double(&grid_cell_delta, transform);
                previous_position.0 = position.0;
            }
        },
    );
}

fn position_to_transform<P: GridPrecision>(
    settings: Res<FloatingOriginSettings>,
    mut origin: Query<
        (&mut Transform, &GridCell<P>, Ref<Position>, &Rotation),
        With<FloatingOrigin>,
    >,
    mut entities: Query<
        (&mut Transform, &mut GridCell<P>, Ref<Position>, &Rotation),
        (Without<FloatingOrigin>, Without<Parent>),
    >,
) {
    let Ok((mut origin_transform, origin_cell, origin_position, origin_rotation)) =
        origin.get_single_mut()
    else {
        warn!("No floating origin found, skipping position to transform sync");
        return;
    };

    origin_transform.translation = origin_position.0.as_vec3();
    origin_transform.rotation = origin_rotation.0.as_f32();

    entities
        .par_iter_mut()
        .for_each(|(mut transform, mut grid_cell, position, rotation)| {
            if origin_position.is_changed() || position.is_changed() {
                // Position is relative to the floating origin's cell so we need to calculate the entity's cell
                // relative to the floating origin's cell.
                let (relative_grid_cell, relative_translation) =
                    settings.translation_to_grid::<P>(position.0);

                // Update grid cell
                let new_grid_cell = *origin_cell + relative_grid_cell;
                if new_grid_cell != *grid_cell {
                    *grid_cell = new_grid_cell;
                }

                if transform.translation != relative_translation {
                    transform.translation = relative_translation;
                }
            }

            transform.rotation = rotation.0.as_f32();
        });
}

/// Updates [`PreviousGlobalTransform`] by setting it to `GlobalTransform` at the very end or start of a frame.
fn update_previous_global_transforms(
    mut bodies: Query<(&GlobalTransform, &mut PreviousGlobalTransform)>,
) {
    for (transform, mut previous_transform) in &mut bodies {
        previous_transform.0 = *transform;
    }
}

/// Computes translation of `Position` based on center of mass rotation and translation
fn get_pos_translation(
    com_translation: &AccumulatedTranslation,
    previous_rotation: &Rotation,
    rotation: &Rotation,
    center_of_mass: &CenterOfMass,
) -> Vector {
    com_translation.0 + previous_rotation.rotate(center_of_mass.0)
        - rotation.rotate(center_of_mass.0)
}
