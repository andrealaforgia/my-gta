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

## The city generator

`tools/common/city_generator.c` holds the settings, the spawn, the map labels,
the routes and `city_generate`. The geometry is in four files that share
`generator_internal.h`: `generator_common.c` (meshes, strips, kerbs, small
vector helpers), `generator_roads.c` (width fitting, junctions, markings,
signs, lamps, manholes, footbridges, roads), `generator_buildings.c` (walls,
roofs, porticoes, landmarks, doors, chimneys, shop fronts) and
`generator_environment.c` (ground, parks, trees, furniture, barriers,
crossings, terrain). Helpers used across files are exported with a `gen_` prefix.

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
  `pedestrians` walks the pavements (adults, children, elders and joggers,
  some in pairs, and some crossing at zebra crossings, where cars stop for
  them). The fleet is fourteen vehicles built from vehicle cfg
  files: four cars for parked and moving traffic, then police, ambulance,
  fire engine, bus and six lorries for moving traffic only. A cfg's `body`
  key picks the shape (`game/src/car/vehicle_bodies.c`); the player can
  drive any of them with V, and the chase camera backs off for long ones.
- **Traffic lights**: signals are `city_signal_t` records (position, the
  heading of the traffic they govern, phase group). Mapped signal nodes give
  two records, one for each way along the road; the generator adds a head at
  the entry of each street of a junction where three streets of tertiary
  rank or better meet (or four meet and two are). Group 0 (east-west) and
  group 1 (north-south) are green in turn on a 48 s cycle; a car stops six
  metres short of a red head it is heading towards.
- **Poles**: every lamp post, signal and road sign is a pole with its own
  numbered parts (`city_pole_source`), which the city scene turns into
  entities and a collider of their own. `poles.c` topples one when the car
  hits it above 2.5 m/s: collider gone, the meshes turn about the foot, the
  light goes dark and a signal stops governing traffic.
- **Sirens**: police, ambulance and fire engine variants are flagged
  `emergency`; about six in ten of those in traffic are on a call, with
  flashing beacons, a little more speed and a two-tone siren the mixer
  swells as the nearest one comes within 150 m. A hard crash jolts the view
  (`shake`), scaled by vehicle mass like the impact sound and the damage.
- **Audio** adds the town (rumble, birds, crickets, cars close by) and a
  church bell on the hour; paused and map screens are silent.
- **Camera views** and a car swap live in the stage; the map is a picture
  rasterised once at load.

## The drive's purpose (2026-10-22)

- **The way to the airport** (`guide_hud.c`): the delivery runs, fares and cash
  are gone. The guide always points at the airport's gate: a pillar of light, the arrow and distance on the HUD, and
  the satnav line and a mark on the minimap and map. The generator joins the
  gate to the nearest drivable junction with "Airport Road"
  (`gen_airport_road`), so the satnav can find it.
- **Satnav** (`navigation.c`): the drivable routes are a graph (junction
  nodes, one-ways respected) searched with Dijkstra. `nav_path` gives the way
  as a polyline for the minimap, map and arrow; `nav_next_hop` tells a police
  car which road to take at its next junction. A target is matched to the
  cheapest reachable road within 40 m of the nearest, because map data has
  one-way spurs and islands; a destination no road leads to is replaced.
- **The law** (`wanted.c`, pursuit in `traffic.c`): a crash felt above 3600
  or a toppled pole is a crime, one star each (at most five, a second apart at
  least); stars send up to three of the police cars that traffic always
  contains (the first three cars of any traffic of twelve or more). Pursuers
  start beyond sight on a road that leads to the player, go 2.6 times the
  usual pace, ignore signals and stop behind the player. Standing still with
  one within 9 m for two seconds is a bust (a quarter of the cash, back to the
  spawn); twelve seconds out of reach once they have found the player, or 90
  without them finding the player, is a getaway.
- **On foot** (`walker.c`): F steps out by the driver's door. The walker has
  no physics body; ground and walls are found with rays (a step above 0.45 m
  is a wall). The car stands with its handbrake on, cars stop for it, and F
  beside it, or beside any parked or moving car, gets in; another car is
  stolen (`parked_cars_take`, `traffic_take_car`), which is a crime.
- **Nothing appears in sight**: traffic is drawn to `TRAFFIC_VISIBLE_RANGE`
  (450 m) and a car is only ever put on the road beyond it and clear of
  other cars.

## Street life (2026-10-22)

The car radio (`engine/audio/radio.c`) is three synthesised stations mixed in `audio_mixer`; E tunes, and it is off on foot. A car at speed that meets a pedestrian knocks them down (`pedestrians_hit`): they slide, lie flat for 25 s, and it counts as a crime. A car at full damage burns for seven seconds and explodes (`step_the_fire`); the player is put back at the start, fined a tenth. The walker climbs into a car in two phases, a walk to the driver's door and a sink into the seat (`step_entry`), and a stolen car keeps its paint (`car_visual_set_paint`, `*_take_car` return it). Shift or Space runs on foot. The fleet has seven cars (hatchback, saloon, van, city car, coupe, SUV, taxi) before the emergency services, buses and lorries; use `FLEET_*_VARIANT` constants rather than numbers.

The generator adds shop signs (`sign` part: a board with a word from a 16-word atlas painted by `texture_signs`) over ground-floor shops, and balconies (`balcony` part, iron on a slab) in front of every other window on plain facades. Both are placed with their building, so the terrain pass skips them (`gen_apply_terrain`). People are dressed with cloth, skin and hair textures through `pedestrian_meshes_set_textures`.

## Textures

`game/src/textures/textures.c` paints every surface at 512 px from periodic
gradient noise (broad and fine variation), cellular noise (stones, tiles,
leaves, gravel) and placed features, with the shading a photograph shows:
asphalt with aggregate, tar-filled cracks, patches and worn paint; slabs and
stone flags with bevelled edges and dirt in the joints; coppi with per-tile
firing, moss in the channels and soot; stucco, painted fresco and brick
facades with sills that drip, louvred shutters, string courses and glass
(alpha 0, which the shader mirrors by day and lights by night); bark,
foliage, rubble masonry, weathered boards and painted metal. Colours are
authored as in a photograph and uploaded as sRGB with anisotropic filtering
(`renderer_upload_surface_texture`), which also keeps dark surfaces from
banding. Each texture is painted once per run and kept for the next stage.

## Physics queries

Static colliders (65 000 in a big city) are kept in a hash grid of 24 m
cells on the ground plane; a ray or a body's contact search lists only the
colliders in the cells its bounds touch, in slot order so ties come out as
they did from a plain scan. Colliders too large to list in 256 cells (and
queries too wide to be worth walking) fall back to the whole list. Freed
slots are reused from a stack. This took a minute of Pordenone with traffic
from about 9 s of CPU to under 2 s, and the load from 11 s to 6 s.

## Streaming

The cooked package is read whole (a few seconds and about 400 MB for the
5 km extract of Pordenone and Cordenons: 1296 chunks, 5.3 million
triangles), but the world only holds what is near the car. `scene_stream`
(`game/src/scenes/city_scene.c`), called every physics step, loads the
chunks within 800 m of the car, nearest first and at most one a step apart
from those within 250 m, and unloads those beyond 1000 m: a chunk's
merged meshes, pole entities and its building and prop colliders. Ground
colliders (roads, pavements, fields) stay for every chunk so cars far away
still drive on something, and routes, signals, crossings and bus stops are
global. Traffic and people more than 900 m and 700 m from the car are moved
to a road or pavement near it, so a big city has them where they can be
seen. The map is a 4096 pixel picture for a big city. Measured at 1080p on
the reference machine: about 4000 draws and 8 ms a frame in town, 14 ms of
CPU for each simulated second with 70 cars and 180 people.

## Rendering

`renderer_draw_mesh` only records a draw. `renderer_end_frame` renders the
list twice: first into a 2048 x 2048 depth map from an orthographic light
box of `shadow_radius` metres around the camera target (front faces
culled, box centre snapped to texels so shadows do not shimmer), then the
main pass into a 4x multisampled floating-point target
(`engine/renderer/gl/gl_post.c`), which is resolved and finished:
screen-space ambient occlusion from the depth buffer (twelve samples in the
hemisphere above each surface, half size, blurred without crossing edges),
bloom from three blurred sizes of whatever is brighter than the screen can
show (the sun, lamps, headlights, lit windows), an ACES tone curve, a little
colour grading and a vignette, then gamma. The HUD is drawn afterwards,
straight to the window. `GTA_NO_POST=1` skips all of this and draws as
before, which is also the fallback if the framebuffers cannot be made.

The surface shader (`engine/renderer/gl/shaders/mesh.frag`, embedded as C
strings into `mesh_fragment.inc` by `scripts/embed_shader.py` at build time)
shades with the sun and a hemisphere of sky above and ground bounce below,
nine rotated shadow samples, variation in the surface itself in
three grains (fading with distance), relief taken from the texture's own
brightness, reflection of the sky on paint, glass and water (with sun glints
and moving waves), window panes that mirror the sky by day and glow by
night, and fog that brightens towards the sun. Draw uniforms are looked up
once per pass, which mattered: 8 000 draws cost 5 ms less. Costs at 1080p on
the reference machine: 9 to 11 ms a frame in town, about 0.3 ms of it for
the post-processing chain. `GTA_DEBUG_SHADOWS=1` draws the shadow factor
instead of colour; `render_snapshot` can dump the shadow map with
`SNAPSHOT_SHADOW_MAP=<file.bmp>`.

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

Re-measured on 2026-10-21 with everything on (110 chunks, about 250k
triangles in the package, 2300 colliders, 60 cars, 150 people, rain, night
lighting, terrain): 2.5 ms per frame in the town centre, 2.5 ms in a narrow
street and 3.8 ms looking across the open ground at the edge, at 1920x1080
with vsync off. Parked cars, people, skid marks and rain drops are culled
by distance on the CPU; the sky, clouds and stars are one full-screen
triangle.

Re-measured on 2026-10-22 on the 5 km Pordenone package (1296 chunks, 5.3
million triangles, 47 000 colliders, streamed): 120 cars and 250 people, about
4 000 draws and 8 to 9 ms per frame at the spawn in the town; 1.1 GB resident.
CPU for the simulation (physics, traffic, people, streaming) is about 30 ms
per simulated second, since the collider grid (see Physics queries); a
minute of the soak run (random driving, walking, stealing cars) costs under
a second. Barletta (700 chunks, 1.3 million triangles) draws in 5 to 9 ms.

## Street life, part two and the airport (2026-10-23)

Pistol on foot (G, click, pad X): `driving_stage_fire`, tracer and flash, `pedestrians_shoot`, `pedestrians_alarm` for the bystanders. Hit people are launched (`launch` in `pedestrians.c`): a rise speed, a flight time and a number of quarter turns, so a car at speed makes them somersault before they land flat, arms and legs thrashing (`body_flail`) in the air and for a couple of seconds after; `pedestrian_t.fall` goes past 1 while they turn and `lift` is the height.

Airports: `tools/common/generator_airport.c` finds a dry site beside the city's extent (`airport_side` in the city config: N, E, S, W, empty to choose, `-` for none), extends the ground to it and lays a 900 m runway with markings, a taxiway with links, an apron, two helicopter pads, a terminal, two hangars and a tower. The package gains `airport` and `aircraft` manifest lines (three planes and two helicopters, with their height).

Flying: `game/src/air/aircraft.c` is the flight model (arcade, no physics world; the caller gives the ground height), `aircraft_visual.c` the shapes. `game/src/stage/air_stage.c` is the integration: F near an aircraft steals it (a crime), the walker is pinned to it so streaming, people and traffic follow, the camera is a level chase camera with a longer near plane, streaming and fog widen, a wall ahead or a hard landing is a crash (the blast, WASTED, the aircraft back on its stand). Keys: W/S pitch, A/D bank, Shift or Space power up, Ctrl power down or brake. The map's edge turns the aircraft back.

Junctions: kerb corners are arcs (`round_corner` in `junction_geometry.c`) when the roads have room (`junction_arm_t.room`), so the roads are cut back a little further.

## The road to the airport (2026-10-24)

The drive starts on "Airport Road" (`spawn_on_airport_road` in `city_generator.c`), about 230 m from the airport's gate, facing it. `gen_airport_road` (in `generator_airport.c`) adds that road to a copy of the model before generation, from the nearest crossing of streets (a junction of degree 3 or more, so the rest of the city reaches it) to the gate, which the manifest records (`gate_x`, `gate_y`, `gate_z` on the `airport` line).

## Bailing out (2026-10-24)

F in the air is a jump (`bail_out` in `air_stage.c`): the player drops with a parachute (`driving_stage_t.chute`: seconds, velocity), 0.7 s of free fall and then 0.9 s for the canopy to open, a descent at about 5.5 m/s, A/D turn and W drifts forward, and the ground or a roof ends it ("LANDED"; an opening too low, over 14 m/s on landing, is WASTED). The aircraft is `doomed`: `aircraft_step` cuts its power and noses it over (a plane) or takes the lift from the rotor (a helicopter), and `air_stage_step_adrift` flies it, with the same wall and ground checks, until it crashes: a blast (`driving_stage_crash_at`, which kills only a player within 9 m) and the aircraft goes back to its stand. A blast from an aircraft does not wreck the player's car (`blast_car`).

The engine note follows the aircraft that is flown: the audio listener distance is taken to it, not to the car left standing at the start, which had made the plane and the helicopter fade away as they climbed.
