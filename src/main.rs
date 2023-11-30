use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy::math::DVec2;
use rand::Rng;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, (
            setup_system,
            spawn_bodies_system,
        ))
        .add_systems(Update, (
            physics_system,
            move_system.after(physics_system),
            cull_system.after(move_system),
        ))
        .run();
}

const G: f64 = 9.0; // Gravitational constant
const DENSITY: f64 = 0.01; // Used to calculate size of bodies, and therefore collision range
const EDGE: f64 = 1500.0; // Extent that bodies can be spawned into, and distance when bodies will be culled
const CENTRAL_MASS: f64 = 10000.0; // Mass of star
const NBODIES: usize = 3000; // Number of non-star bodies
const BODY_MASS: f64 = 1.0;

#[derive(Component, Copy, Clone)]
struct Body {
    vel: DVec2,
    pos: DVec2,
    size: f64,
    mass: f64,
    dead: bool,
}

impl Body {
    fn new(vel: DVec2, pos: Option<DVec2>, mass: f64) -> Body {
        let default_pos = DVec2::new(0.0, 0.0); // Replace with your default origin
        let size = ((3.0 * mass) / (4.0 * std::f64::consts::PI * DENSITY)).powf(1.0 / 3.0);

        Body {
            vel,
            pos: pos.unwrap_or(default_pos),
            size,
            mass,
            dead: false,
        }
    }
}

fn get_size(mass: f64) -> f64 {
    ((3.0 * mass) / (4.0 * std::f64::consts::PI * DENSITY)).powf(1.0 / 3.0)
}

#[derive(Component)]
struct Cull(Reason);

#[derive(Debug)]
enum Reason {
    Collision,
    Escape,
}

fn setup_system (
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Spawn camera
    commands.spawn(Camera2dBundle::default());

    // Spawn visual ring for EDGE of universe
    let segments = 64;
    let mut positions = Vec::new();
    let mut indices = Vec::new();

    for i in 0..segments {
        let angle = i as f32 * 2.0 * std::f32::consts::PI / segments as f32;
        positions.push([EDGE as f32 * angle.cos(), EDGE as f32 * angle.sin(), 0.0]);
        indices.push(i);
    }
    indices.push(0); // Connect the last point to the first

    let mut ring = Mesh::new(bevy::render::render_resource::PrimitiveTopology::LineStrip);
    ring.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    ring.set_indices(Some(bevy::render::mesh::Indices::U32(indices)));

    commands.spawn(MaterialMesh2dBundle {
        mesh: meshes.add(ring).into(),
        material: materials.add(ColorMaterial::from(Color::WHITE)),
        ..default()
    });
}

fn spawn_bodies_system (
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Spawn central mass
    commands.spawn((
        Body::new(DVec2 { x: 0.0, y: 0.0 }, None, CENTRAL_MASS),
        MaterialMesh2dBundle {
            mesh: meshes.add(shape::Circle::new(get_size(CENTRAL_MASS) as f32).into()).into(),
            material: materials.add(ColorMaterial::from(Color::BLACK)),
            transform: Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
            ..default()
        },
    ));

    // Spawn body positions, velocities, and meshes
    let size = get_size(BODY_MASS) as f32;
    let offset = get_size(CENTRAL_MASS) * 4.0;
    let spawn_radius = EDGE - 50.0; // Outer radius for spawning
    let mut rng = rand::thread_rng();
    
    // Vectors to store positions and velocities
    let mut positions = Vec::new();
    let mut velocities = Vec::new();
    
    // Step 1: Generate all positions
    for _ in 0..NBODIES {
        // Generate position
        let angle = rng.gen::<f64>() * 2.0 * std::f64::consts::PI;
        let radius_squared = rng.gen::<f64>() * (spawn_radius.powi(2)) + offset.powi(2);
        let radius = radius_squared.sqrt();
        let pos = DVec2 {
            x: radius * angle.cos(),
            y: radius * angle.sin(),
        };
        positions.push(pos);
    }
    
    // Step 2: Generate all velocities

    let mut gravitational_forces = vec![DVec2::new(0.0, 0.0); NBODIES];

    // Calculate gravitational forces
    for (i, pos_i) in positions.iter().enumerate() {
        // Force from the central mass
        let direction_to_central_mass = -(*pos_i); // Direction to the central mass (origin)
        let distance_to_central_mass_squared = direction_to_central_mass.length_squared();
        let central_mass_force_magnitude = G * CENTRAL_MASS * BODY_MASS / distance_to_central_mass_squared;
        gravitational_forces[i] += direction_to_central_mass.normalize() * central_mass_force_magnitude;
    
        // Forces from other bodies
        for (j, pos_j) in positions.iter().enumerate() {
            if i != j {
                let direction = *pos_j - *pos_i;
                let distance_squared = direction.length_squared();
                let force_magnitude = G * BODY_MASS * BODY_MASS / distance_squared;
                gravitational_forces[i] += direction.normalize() * force_magnitude;
            }
        }
    }
    
    for (force, pos) in gravitational_forces.iter().zip(positions.iter()) {
        let distance = pos.length();
        let force_magnitude = force.length();
        
        // Calculate the required speed for a stable orbit considering the net force
        let required_speed = (force_magnitude * distance / BODY_MASS).sqrt();
    
        // Ensure the velocity is perpendicular to the net force vector
        let vel_direction = DVec2::new(-force.y, force.x).normalize();
        let vel = vel_direction * required_speed;
    
        velocities.push(vel);
    }
    
    // Step 3: Spawn each body using the stored positions and velocities
    for (pos, vel) in positions.iter().zip(velocities.iter()) {
        commands.spawn((
            Body::new(*vel,Some(*pos),BODY_MASS),
            MaterialMesh2dBundle {
                mesh: meshes.add(shape::Circle::new(size).into()).into(),
                material: materials.add(ColorMaterial::from(Color::BLACK)),
                transform: Transform::from_translation(Vec3::new(pos.x as f32, pos.y as f32, 0.0)),
                ..default()
            },
        ));
    }
    
}

fn physics_system(
    mut commands: Commands,
    mut q: Query<(Entity, &mut Body, &mut Transform)>,
    time: Res<Time>,
) {
    let mut iter = q.iter_combinations_mut();
    while let Some([(
        _entity_a,
        mut body_a,
        mut transform_a,
    ), (
        entity_b,
        mut body_b,
        mut _transform_b,
    )]) = iter.fetch_next() {
        if !body_a.dead && !body_b.dead {
            let direction = body_b.pos - body_a.pos;
            let distance = direction.length();

            // Skip the force calculation if bodies are "colliding"
            if distance < (body_a.size + body_b.size) {
                commands.entity(entity_b).insert(Cull(Reason::Collision));
                body_b.dead = true;

                let new_mass = body_a.mass + body_b.mass;
                let new_pos = DVec2 { 
                    x: (body_a.pos.x * body_a.mass + body_b.pos.x * body_b.mass) / new_mass, 
                    y: (body_a.pos.y * body_a.mass + body_b.pos.y * body_b.mass) / new_mass
                };
                let weighted_vel1 = body_a.vel * body_a.mass;
                let weighted_vel2 = body_b.vel * body_b.mass;

                let scale = get_size(new_mass) / body_a.size;

                body_a.vel = (weighted_vel1 + weighted_vel2) / new_mass;
                body_a.pos = new_pos;
                body_a.size = get_size(new_mass);
                body_a.mass = new_mass;

                transform_a.translation.x = new_pos.x as f32;
                transform_a.translation.y = new_pos.y as f32;

                let old_scale = transform_a.scale;
                transform_a.scale *= Vec3::splat(scale as f32);
                if transform_a.scale.x < old_scale.x { println!("old scale {:?}, new scale {:?}", old_scale, transform_a.scale)};

                continue;
            }

            if distance > 0.0 {
                let force_magnitude = G * (body_a.mass * body_b.mass) / (distance * distance);
                let force_direction = direction.normalize();

                // Apply force to body_a
                let acceleration_a = force_direction * force_magnitude / body_a.mass;
                body_a.vel += acceleration_a * time.delta_seconds_f64();

                // Apply force to body_b (in the opposite direction)
                let acceleration_b = -force_direction * force_magnitude / body_b.mass;
                body_b.vel += acceleration_b * time.delta_seconds_f64();
            }
        }
    }
}

fn move_system(
    mut commands: Commands,
    mut q: Query<(Entity, &mut Body, &mut Transform)>,
    time: Res<Time>,
) {
    for (e, mut body, mut transform) in &mut q {
        if body.dead {
            continue;
        }

        let vel = body.vel;
        body.pos += vel * time.delta_seconds_f64();
        transform.translation = Vec3 { x: body.pos.x as f32, y: body.pos.y as f32, z: 0.0 };

        // Calculate the distance from the origin
        let distance_from_origin = body.pos.length(); // This calculates the Euclidean distance

        if distance_from_origin > EDGE {
            commands.entity(e).insert(Cull(Reason::Escape));
        }
    }
}

fn cull_system(
    mut commands: Commands,
    q: Query<(Entity, With<Cull>)>,
) {
    for (e, _) in q.iter() {
        commands.entity(e).despawn();
    }
}
