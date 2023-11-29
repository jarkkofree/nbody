# Gravitational Simulation in Bevy

## Overview
This project is a 2D gravitational simulation built using the Bevy game engine. It demonstrates the interaction of celestial bodies under the influence of gravity, including collisions and orbits around a central mass.

## Features
- **Simulation of Gravitational Forces**: Models gravitational interactions between multiple bodies.
- **Collision Handling**: Detects and handles collisions between celestial bodies, including absorption and mass increase.
- **Central Mass Influence**: Includes a large central mass influencing the movement of other bodies.
- **Edge Culling**: Bodies that move beyond the defined universe edge are removed from the simulation.
- **Dynamic Body Creation**: Allows the dynamic creation of bodies with varying masses and initial velocities.

## How to Run
Ensure you have Rust and the Bevy engine installed. Clone the repository and run the project using Cargo:

```bash
cargo run --release
```

## Implementation Details
- **Physics System**: Handles the calculation of gravitational forces and applies them to each body.
- **Movement System**: Updates the positions of bodies based on their velocities.
- **Culling System**: Removes bodies that collide or move beyond the universe edge.
- **Body Spawning**: Generates bodies with random positions and velocities within a defined space.

## Contributing
Contributions are welcome! If you have ideas for improvements or find a bug, please open an issue or a pull request.
