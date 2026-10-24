# Architecture

## Repeatable frame benchmark (2026-10-24)

`./gta-like --bench --city=pordenone` runs a fixed 1800-frame camera route
after 60 warm-up frames, with two 1/120 s simulation steps per frame. These
are 30 measured simulated seconds. Loading and warm-up are excluded. Vsync
and audio are disabled. The camera orbits the city centre, at most 750 m out
and 80 m above the highest chunk, looking down towards nearby ground; the
walking player's focus follows it so streaming and street life follow too.

`game/src/main/benchmark.c` owns measurement, nearest-rank p99, strict report
I/O and the 25% regression gate. The renderer's `renderer_wait_idle` waits
for completed drawing with `glFinish`, used only by the benchmark; ordinary
game frames retain their usual pipeline. Frame time includes simulation,
rendering, GPU completion and swap. Simulation time includes stage systems
and streaming, a deliberate superset of physics-solver time. Draw calls and
triangles include both the shadow and main passes.

`--bench-output=<file>` saves a config-format report. Supplying
`--bench-baseline=<file>` refuses incompatible workloads, then fails when any
reported metric exceeds 125% of the reference. Identity covers resolution,
sample/warm-up counts, actor counts, city/config/geometry and simulation
metadata, effective vehicle parameters, time/weather and active rendering
modes. Native geometry representations make these same-machine references.
No universal GPU timing gate runs on hosted CI; controlled unit and runner
tests verify the comparison, with CLI tests covering option validation and
protection against overwriting the baseline through aliases.

Measured on Apple M4 Pro, 24 GB, macOS 26.5.2, OpenGL 4.1 Metal 90.5 at
1280 by 720, dry daylight, post-processing enabled, 120 cars and 250 people.
The current Pordenone package has 1368 chunks and 7,157,082 triangles;
config hash `fc89e80665356d89`. The reference is
`docs/baselines/pordenone-m4-pro.cfg`.

| Metric | Reference |
|---|---:|
| Mean completed frame | 12.980 ms |
| 99th-percentile completed frame | 19.275 ms |
| Mean simulation per frame | 0.775 ms |
| Mean draw calls | 8121.93 |
| Mean triangles | 909576.88 |

The mean is about 77 frames/s; the p99 exceeds the 16.67 ms budget for a
consistently smooth 60 frames/s. Intel integrated graphics remain unmeasured.
These are observations, not an assurance about every street or machine.
Route clearance, radius and look direction are named tuning constants.

Viewed the route at its start and after a quarter orbit. Streets, buildings,
parks, the river and traffic are visible; aiming at nearby ground replaced
the initial view's excess sky and fog. Distant fog remains strong and the
HUD still describes the parked car near the airport; those are unchanged
game behaviours. Next is roadmap 0.3, input recording and deterministic replay.

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

Punch on foot (Q, right click, pad B): `driving_stage_punch` and `pedestrians_punch`: the nearest person within 1.5 m and sixty degrees reels for half a second (`stagger`, drawn as a lean with the arms flung) and then runs away from the puncher at the panic pace, turning to go the other way along the pavement; people within 9 m take fright too. The right arm swings forward in the punch (`punch` in `pedestrian_draw_figure_fallen`). No crime.

Pistol on foot (G, click, pad X): `driving_stage_fire`, tracer and flash, `pedestrians_shoot`, `pedestrians_alarm` for the bystanders. Hit people are launched (`launch` in `pedestrians.c`): a rise speed, a flight time and a number of quarter turns, so a car at speed makes them somersault before they land flat, arms and legs thrashing (`body_flail`) in the air and for a couple of seconds after; `pedestrian_t.fall` goes past 1 while they turn and `lift` is the height.

Airports: `tools/common/generator_airport.c` finds a dry site beside the city's extent (`airport_side` in the city config: N, E, S, W, empty to choose, `-` for none), extends the ground to it and lays a 900 m runway with markings, a taxiway with links, an apron, two helicopter pads, a terminal, two hangars and a tower. The package gains `airport` and `aircraft` manifest lines (three planes and two helicopters, with their height).

Flying: `game/src/air/aircraft.c` is the flight model (arcade, no physics world; the caller gives the ground height), `aircraft_visual.c` the shapes. `game/src/stage/air_stage.c` is the integration: F near an aircraft steals it (a crime), the walker is pinned to it so streaming, people and traffic follow, the camera is a level chase camera with a longer near plane, streaming and fog widen, a wall ahead or a hard landing is a crash (the blast, WASTED, the aircraft back on its stand). Keys: W/S pitch, A/D bank, Shift or Space power up, Ctrl power down or brake. The map's edge turns the aircraft back.

Junctions: kerb corners are arcs (`round_corner` in `junction_geometry.c`) when the roads have room (`junction_arm_t.room`), so the roads are cut back a little further.

## The road to the airport (2026-10-24)

The drive starts on "Airport Road" (`spawn_on_airport_road` in `city_generator.c`), about 230 m from the airport's gate, facing it. `gen_airport_road` (in `generator_airport.c`) adds that road to a copy of the model before generation, from the nearest crossing of streets (a junction of degree 3 or more, so the rest of the city reaches it) to the gate, which the manifest records (`gate_x`, `gate_y`, `gate_z` on the `airport` line).

## Bailing out (2026-10-24)

F in the air is a jump (`bail_out` in `air_stage.c`): the player drops with a parachute (`driving_stage_t.chute`: seconds, velocity), 0.7 s of free fall and then 0.9 s for the canopy to open, a descent at about 5.5 m/s, A/D turn and W drifts forward, and the ground or a roof ends it ("LANDED"; an opening too low, over 14 m/s on landing, is WASTED). The aircraft is `doomed`: `aircraft_step` cuts its power and noses it over (a plane) or takes the lift from the rotor (a helicopter), and `air_stage_step_adrift` flies it, with the same wall and ground checks, until it crashes: a blast (`driving_stage_crash_at`, which kills only a player within 9 m) and the aircraft goes back to its stand. A blast from an aircraft does not wreck the player's car (`blast_car`).

The engine note follows the aircraft that is flown: the audio listener distance is taken to it, not to the car left standing at the start, which had made the plane and the helicopter fade away as they climbed.

## Doors, drivers, parachute and background sound (2026-10-24)

Cars have a driver's door (`car_visual_draw_door`, hinged at its front edge on the left: panel, window and a dark doorway built in `car_visual_create`; lorries and buses get it on the cab). `door_open_amount` in `driving_stage.c` swings it out as the person reaches it, holds it while they get in and shuts it behind them (`entering`), and on the way out opens, holds and shuts it (`door_clock`). Taking a car off the road (`traffic_take_car`) hauls its driver out: `pedestrians_eject_driver` puts the person furthest from the car on the pavement beside it, reeling, then running.

The parachute is closer to the person (camera 5 m back), the figure falls belly down, thrashing, for 0.7 s (`air_stage_chute_pose`), the canopy opens with a jolt (`audio_device_trigger_impact`) and sways like a pendulum, leaning into turns; holding S, or the last 3.5 m, flares for a gentler landing; the HUD shows height and descent rate. The wind is `audio_state_t.wind`: a mid-band rush that is there at any speed and grows with the fall, heard close to (`listener_distance` to the camera).

The background sound has no birds and no crickets any more (their pure 2 to 4 kHz tones rang like a bell); `tests/test_audio.c` checks that no pitch stands out of the town's hum, day or night. The yellow pillar and ring over the airport gate are gone; the arrow, distance and map marks remain.

## More aircraft, cockpits and missiles (2026-10-24)

`aircraft_kind_t` grew at the end (packages store the number): airliner (a
Boeing 747), jet fighter and twin-rotor gunship. Fixed-wing flight reads its
numbers from the `wings` table in `aircraft.c` (stall and lift-off speed,
thrust, drag, pitch and roll rates, how hard a banked turn bites); the
gunship is the helicopter model with a pace of 1.45. `aircraft_models.c`
builds the shapes from swept slabs, ellipsoids and boxes; `aircraft_cockpit.c`
builds a flight deck per kind and draws it in the aircraft's frame, with a yoke
or stick that follows the controls and a roll indicator. C while flying
toggles `air_view` between chase and cockpit; the hull is not drawn from the
seat, only what spins (propeller, rotors). The horn sounds from an aircraft.

The jet and the gunship carry missiles. G (`driving_stage_fire`) launches one
from a rail under a wing (cooldown 0.45 s, eight in flight at most, a crime
each time). A missile flies straight at 190 m/s on top of the aircraft's speed,
falls a little, and goes off where a ray along its last step meets anything
(ground, building, car). `strike` draws the fireball (the car-blast
visual), throws every person within 12 m (`pedestrians_blast`), and removes
traffic cars in the blast with `traffic_take_car`. A gunsight ring floats
120 m ahead of the nose of an armed aircraft.

The airport now stands eight aircraft: three planes, two helicopters, the
747 on the taxiway, the jet on the apron and the gunship on a third pad.
Pessano con Bornago (the plain east of Milan) joined Pordenone and Barletta as
a cooked city (`make new-city`, provenance in `data/`).

## Headless city soak (2026-10-24)

`make soak` runs the existing citydrive group under ASan and UBSan with a
stub renderer, dummy SDL drivers and allocation tracking. Its busy-city test
cooks the hand-written fixture, starts 20 traffic cars and 40 pedestrians,
and drives for 68 simulated seconds, including reversing. It now checks
that the player travels at least 10 m from the start and that traffic and
pedestrian positions, headings and speeds stay finite. The player's height
and position are checked every physics step. This uses the real stage,
physics, traffic, pedestrian and package-loading code without a GPU.

The `Headless city soak` Actions workflow runs daily at 03:23 UTC and can
also be dispatched manually, on both Ubuntu and macOS. Assertions,
sanitizer errors, allocation leaks and test timeouts fail the job. The
usual push workflow still runs the full test and sanitizer suites.

Viewed a fixture snapshot after three seconds with 20 cars and 40 people:
the car, streets, traffic, figures, minimap and speedometer render correctly.
The bottom control hint is hard to read against pale ground; left for the
UI pass. This slice changes no gameplay or rendering. Handling and feel
still need play-testing. Next is roadmap 0.2: a repeatable camera-path
benchmark with average and 99th-percentile frame times and a recorded budget.

## Instruments, landing gear and the missile sound (2026-10-24)

Every flight deck now has real instruments, drawn as round dials in the
aircraft's frame (`aircraft_cockpit.c`): airspeed (needle sweeps 270 degrees to
a per-kind full scale), altitude over the ground (a long needle once round a
thousand metres, a short one for ten thousand), a horizon ball that is blue
above and brown below and is turned by the inverse of the aircraft's pitch and
roll, and a compass card whose letters are made from the built-in 5x7 font and
turn against the heading (`-yaw`; the nose points north at yaw 0). Aeroplanes
add three gear lights (red while the wheels move, green when down and locked,
dark when up) and a lever. The 747 has a captain's and a first officer's set,
six windscreen panes, a pedestal with four throttle levers, engine bars and two
yokes. The numeric HUD is hidden from the seat; the cockpit camera glances a
little down so the panel is in view. A quad with a repeated corner has a zero
normal and renders as garbage: use `mesh_data_add_triangle` for triangles.

`aircraft_t` gained `gear_down` and `gear` (1 down .. 0 up, four seconds to
move). L toggles it (`air_stage_toggle_gear`) but only in the air; the wheels
are their own mesh and fold into the belly; gear up cuts a sixth of the drag;
touching down with `gear < 0.8` is a crash. A boarding walker is measured to
the aircraft's centre line, not its middle (`distance_to_body`), so the nose of
a 747 is in reach.

`audio_mixer_trigger_missile` makes the launch: a thump falling from about 100
to 36 Hz, a noise roar that starts bright and darkens as the missile draws away
(two one-pole filters whose coefficient falls over two seconds), and a rumble
of three low notes, fading over about four seconds. Nothing in it is a pure
tone above 100 Hz.

## The player on the map, and a picked-out place (2026-10-24)

The map, the minimap and the on-screen arrows follow `player_position` and
`player_heading` in `driving_stage.c`: the aircraft's nose while flying, the
walker on foot or on a parachute, the car otherwise. (They used the car's body
before, so they showed the car left at the airport while the player flew a
helicopter, and the walker's car while on foot.)

`guide_t` now has a label and a colour. A city may have a file
`assets/targets/<package folder>.txt` with one line `latitude longitude NAME`;
the game puts the place on the ground using the package's origin
(`guide_target_load`), shows a red diamond on the minimap and the map with its
name, and an arrow and distance under the airport's, which points straight at it
by road or by air. The only one so far is Corso Europa 22 in Pessano con
Bornago (there is no number 22 in the map data, so it sits between 18 and 26);
it is temporary: delete the file to remove it.
