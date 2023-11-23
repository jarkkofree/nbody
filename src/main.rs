use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy::math::DVec2;
use rand::Rng;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup_system)
        .add_systems(Update, (
            physics_system,
            move_system.after(physics_system),
            collision_system.after(move_system),
            cull_system.after(collision_system),

        ))
        .run();
}

#[derive(Component, Copy, Clone)]
struct Body {
    vel: DVec2,
    pos: DVec2,
    mass: f64,
}

fn setup_system (
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn(Camera2dBundle::default());

    let mass = 1.0;
    let size = mass as f32;
    let vel_range = 10.0;
    let pos_range = 100.00;
    let mut rng = rand::thread_rng();
    
    for _ in 0..10 {

        let vel = DVec2 { x: rng.gen::<f64>()*vel_range - vel_range/2.0, y: rng.gen::<f64>()*vel_range - vel_range/2.0 };
        let pos = DVec2 { x: rng.gen::<f64>()*pos_range - pos_range/2.0, y: rng.gen::<f64>()*pos_range - pos_range/2.0 };

        let e = commands.spawn((
            Body {
                vel: vel,
                pos: pos,
                mass: mass,
            },
            MaterialMesh2dBundle {
                mesh: meshes.add(shape::Circle::new(size).into()).into(),
                material: materials.add(ColorMaterial::from(Color::WHITE)),
                transform: Transform::from_translation(Vec3::new(pos.x as f32, pos.y as f32, 0.0)),
                ..default()
            },
        )).id();
        println!("spawning e: {:?}", e);

    }
}

const G: f64 = 1.0e2; // Gravitational constant

fn physics_system(
    mut q: Query<(Entity, &mut Body)>,
    time: Res<Time>,
) {
    let bodies: Vec<(Entity, Body)> = q.iter_mut().map(|(e, b)| (e, *b)).collect();

    for (entity_a, mut body_a) in q.iter_mut() {
        let mut total_force = DVec2::ZERO;

        for (entity_b, body_b) in bodies.iter() {
            if entity_a == *entity_b {
                continue;
            }

            let direction = body_b.pos - body_a.pos;
            let distance = direction.length();
            let force_magnitude = G * (body_a.mass * body_b.mass) / (distance * distance);

            total_force += direction.normalize() * force_magnitude;
        }

        let acceleration = total_force / body_a.mass;
        body_a.vel += acceleration * time.delta_seconds_f64();
    }
}

#[derive(Component)]
struct Cull;

fn move_system(
    mut commands: Commands,
    mut q: Query<(Entity, &mut Body, &mut Transform)>,
    time: Res<Time>,
) {
    for (e, mut body, mut transform) in &mut q {
        let vel = body.vel;
        body.pos += vel * time.delta_seconds_f64();
        transform.translation = Vec3 { x: body.pos.x as f32, y: body.pos.y as f32, z: 0.0 };

        if body.pos.x > 500.0 || body.pos.y > 500.0 {
            commands.entity(e).insert(Cull);
            println!("culling {:?} due to pos {:?}", e, body.pos);
        }

    }
}

fn collision_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    q: Query<(Entity, &Body)>,
) {
    for (entity_a, body_a) in q.iter() {
        for (entity_b, body_b) in q.iter() {
            if entity_a == entity_b {
                continue;
            }

            if body_a.mass < body_b.mass {
                continue;
            }

            let direction = body_b.pos - body_a.pos;
            let distance = direction.length();

            if distance < body_a.mass {
                if body_a.mass > body_b.mass || (body_a.mass == body_b.mass && entity_a > entity_b) {
                    
                    let total_mass = body_a.mass + body_b.mass;
                    let weighted_vel1 = body_a.vel * body_a.mass;
                    let weighted_vel2 = body_b.vel * body_b.mass;

                    // With this code disabled, the rest of the code behaves as expected. Although no new objects are created,
                    // collisions continue to happen and Bodys are despawned in pairs as expected.
                    //
                    // When this code is enabled, the first time it runs, the screen goes to ClearColor and all objects dissappear
                    // and no more collisions take place
                    
                    println!("about to spawn a new body due to collision, vel: {:?}, pos: {:?}, mass: {:?}", (weighted_vel1 + weighted_vel2) / total_mass, body_a.pos, total_mass);
                    let e = commands.spawn((
                        Body {
                            vel: (weighted_vel1 + weighted_vel2) / total_mass,
                            pos: body_a.pos,
                            mass: total_mass,
                        },
                        MaterialMesh2dBundle {
                            mesh: meshes.add(shape::Circle::new(total_mass as f32).into()).into(),
                            material: materials.add(ColorMaterial::from(Color::WHITE)),
                            transform: Transform::from_translation(Vec3::new(body_a.pos.x as f32, body_a.pos.y as f32, 0.0)),
                            ..default()
                        },
                    )).id();
                    println!("new body: {:?}", e);
                    
                    // End problem code

                    commands.entity(entity_a).insert(Cull);
                    commands.entity(entity_b).insert(Cull);

                    println!("despawning {:?} and {:?} due to collision", entity_a, entity_b);
                }
            }

        }
    }
}

fn cull_system(
    mut commands: Commands,
    q: Query<Entity, With<Cull>>,
) {
    for e in q.iter() {
        commands.entity(e).despawn();
    }
}