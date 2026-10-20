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
| `engine/renderer` | the renderer interface, CPU meshes, materials, the 5x7 font; `gl/` is the only OpenGL code: a deferred draw list, a shadow-map pass and the main pass | core, math |
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

## Game systems (2026-10-21)

- **Time of day**: `game/src/stage/daylight.c` turns an hour into the sun and
  moon's direction, light colour, ambient, horizon and zenith colours, star
  level and lamp glow, continuously. The stage advances the hour, lights
  `lamp_light` parts and, through `window_glow`, the panes that textures mark
  with alpha 0, and the player's headlight cone (one spot in the shader).
- **Cars about town**: the generator writes `route` and `park` records; the
  scene keeps them. `parked_cars` stands cars at the kerb, `traffic` drives
  kinematic cars (box colliders moved each step) on `route_path` lanes with
  right-hand lanes, junction right of way and stopping for the player,
  `pedestrians` walks the pavements. A fleet of four bodies is built from
  vehicle cfg files.
- **Audio** adds the town (rumble, birds, crickets, cars close by) and a
  church bell on the hour; paused and map screens are silent.
- **Camera views** and a car swap live in the stage; the map is a picture
  rasterised once at load.

## Rendering

`renderer_draw_mesh` only records a draw. `renderer_end_frame` renders the
list twice: first into a 2048 x 2048 depth map from an orthographic light
box of `shadow_radius` metres around the camera target (front faces
culled, box centre snapped to texels so shadows do not shimmer), then the
main pass with one directional light, hemisphere ambient, a 3 x 3 PCF
shadow lookup, metallic/roughness shading and distance fog, followed by
debug lines and text. `GTA_DEBUG_SHADOWS=1` in the environment draws the
shadow factor instead of colour; `render_snapshot` can dump the shadow
map with `SNAPSHOT_SHADOW_MAP=<file.bmp>`.

## Conventions

See `docs/decisions/ADR-007-coordinate-conventions.md`: +X east, +Y up,
+Z south, forward is -Z, metres, seconds, radians, column-major matrices,
anticlockwise front faces.

## Performance baseline (2026-10-20)

Reference machine: Apple M4 Pro, 24 GB, macOS 26.5, 1920x1080 drawable.
`render_snapshot` on the full Pordenone package (90 chunks, about 100k
triangles with trees, 2300 colliders, shadows on): about 700 draw calls
across the shadow and main passes, 130k triangles drawn at a street-level
view, 1.8 to 2.3 ms per frame measured over 60 frames with vsync off.
The physics step with the car is well under a millisecond. No instancing,
LOD or chunk streaming yet; every chunk is loaded at start.
