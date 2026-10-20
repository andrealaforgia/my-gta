# Architecture

A real-time engine, an offline content pipeline and a thin prototype
layer, in C99 with SDL2 for the platform and OpenGL 4.1 for drawing.

```
OSM XML extract --> city-import --> intermediate model --> city-generate --> cooked package --> city-validate
                                                                                   |
                                                                                   v
                     game (prototype)  -->  engine: platform, renderer, world, physics, vehicle, camera, audio
```

## Modules and dependency direction

| Module | Owns | Depends on |
|---|---|---|
| `engine/core` | logging, key = value configuration | libc |
| `engine/math` | vectors, quaternions, matrices, transforms, boxes, rays, frustums, damping | libc |
| `engine/platform` | window and GL context, events, keyboard and controller mapping, fixed-step frame clock, clock | core, SDL2 |
| `engine/renderer` | the renderer interface, CPU meshes, materials, the 5x7 font; `gl/` is the only OpenGL code | core, math |
| `engine/assets` | the cooked city package reader and writer | renderer (mesh data), math |
| `engine/world` | entities with transforms and bounds, frustum culling, drawing | renderer, math |
| `engine/physics` | box bodies, static box and mesh colliders, ray casts, contacts, stepping | core, math |
| `engine/audio` | the pure mixer (engine, skid, impact, wind) and the SDL device | SDL2 |
| `engine/vehicle` | raycast wheels, suspension, tyres, drivetrain, gearbox, parameters | physics, platform (input filter), core |
| `engine/camera` | chase camera with geometry avoidance | physics, renderer (camera struct) |
| `game/` | scenes (test track, city loader), car visuals, textures, the driving stage, main | everything above |
| `tools/common` | OSM reader, projection, intermediate model, importer, triangulator, generator, validator | core, math, renderer (mesh data), assets |

Nothing in `engine/` includes `game/` or `tools/`. The runtime never sees
OSM data; it loads the cooked package. OpenGL is confined to
`engine/renderer/gl/gl_renderer.c`; tests link a stub renderer instead.

## Runtime

`driving_stage_run` is the loop: poll the window, read input, advance the
frame clock, run the fixed steps (`vehicle_update` then
`physics_world_step` at 1/120 s, catch-up capped at 250 ms), update the
chase camera, render the interpolated state, swap. The audio device runs
its own callback thread fed with a state snapshot per frame under the
device lock.

Ownership is explicit: the stage owns the physics world, the world, the
scene's meshes and the vehicle; `main` owns the window, renderer and
audio device and passes pointers down. No globals.

## Conventions

See `docs/decisions/ADR-007-coordinate-conventions.md`: +X east, +Y up,
+Z south, forward is -Z, metres, seconds, radians, column-major matrices,
anticlockwise front faces.

## Performance baseline (2026-10-20)

Reference machine: Apple M4 Pro, 24 GB, macOS 26.5, 1920x1080 drawable.
`render_snapshot` on the Pordenone package (80 chunks, 25k triangles,
380 meshes, 1672 colliders): 309 draw calls and 22k triangles after
frustum culling, 4.9 ms per frame measured over 60 frames with vsync off.
The physics step with the car is well under a millisecond. No instancing,
LOD or chunk streaming yet; every chunk is loaded at start.
