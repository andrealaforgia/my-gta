# Roadmap: making gta-like more and more like Grand Theft Auto

This is a brief for an autonomous coding agent (Codex) that will carry the
game forward over many sessions. Read it all once, then work through the
phases in order. Each phase is a list of small, independently shippable
slices. Do not start a slice until the previous one is merged, green and
pushed.

The goal is a single-player, open-world, third-person crime game in the
spirit of GTA III and Vice City: a real city to drive, fly and walk through,
a living street, a police response that escalates, missions that give the
player something to do, and money to do it with. It is not a clone of any
game, and it must not use any Rockstar asset, name, map or story.

## 1. Ground rules (non-negotiable)

**The owner's principles.** Test-driven development, trunk-based
development, small commits, lean dependencies, clean code. Write the failing
test first, make it pass, then refactor. Commit to `main` after every green
slice and push straight away. There are no long-lived branches and no pull
request queue. A slice should be a few hours of work at most; if it is
bigger, split it.

**Language and stack.** C99 only, SDL2, OpenGL 4.1 core, plain `make`,
Python 3 for scripts and the test runner. Do not add a dependency, a
language or a build system without writing a short decision record in
`docs/decisions/` that says what it replaces and what it costs. Prefer
writing fifty lines to adding a library. Standard library only for tools.

**Code style.** Match the surrounding code: naming, comment density, idiom.
`clang-format` per file (`clang-format -i path.c`, never the whole tree in
one go, and never reformat GLSL embedded in C strings). `cpplint` must pass
(`make lint`): 80 columns, no empty loop bodies, braces on multi-statement
bodies, no multi-line block comments in headers. Compiler flags are
`-Wall -Wextra -pedantic-errors`; tests also run with `-Werror`.

**Definition of done for every slice.**

1. A test that failed before the change and passes now. Tests live in
   `tests/test_*.c`, are grouped in `tests/test_groups.h` and run by
   `python3 tests/run_tests.py`.
2. `make lint`, `make -j4`, `make test`, `make sanitize` all green on the
   Mac and on Linux (see section 2 on gcc). Never commit on red.
3. If the change is visible, a snapshot rendered with `render_snapshot` and
   looked at (see section 3). Tests prove behaviour; only eyes prove it looks
   right.
4. `docs/architecture.md` gets a dated section if a module, a rule or a
   data format changed. `README.md` gets the key bindings if input changed.
5. One commit per slice with a message that says what the player sees, then
   `git push`. Commits end with the attribution line your host requires.

**Safety and privacy.**
- Never put a personal email address, a name or any credential in a header,
  a URL, a payload, a file or a commit. The HTTP user agent for every
  download is exactly `gta-like-prototype/0.1`.
- Download politely: one request a second, tiles of a few hundredths of a
  degree, once. Respect each service's terms.
- Do not commit raw map extracts, elevation grids, LiDAR, cooked cities or
  anything under `data/` or `assets_cooked/` (they are gitignored). Do
  commit the small `PROVENANCE.md` file beside each extract.
- Every third-party asset (model, texture, sound, font, music) needs a
  licence that allows redistribution in a game, recorded in
  `THIRD_PARTY_NOTICES.md` with source, author, licence and date. When in
  doubt, generate it procedurally, as the project already does for meshes,
  textures and sound. Map data is OpenStreetMap, ODbL 1.0, attribution
  required in the game's credits and README.

**Do not break these standing behaviours.**
- Cars never appear from nothing, and neither do people: anything that
  spawns does so out of sight, beyond `TRAFFIC_VISIBLE_RANGE` or behind a
  corner, and recycling happens out of view.
- Trucks, buses and long vehicles must not get stuck on corners.
- A stolen car keeps its colour.
- No high ringing tones in the background audio (a pure 2 to 4 kHz tone
  sounds like a bell). New ambient sound must pass the spectrum test in
  `tests/test_audio.c`.
- Every new vehicle in the fleet must be stealable and drivable
  (`every_fleet_vehicle_can_be_stolen_and_driven`).

## 2. Building, testing and the two compilers

```
export PATH=/opt/homebrew/bin:$PATH    # macOS, Homebrew
make -j8                  # the game: ./gta-like
make tools                # city-import, city-generate, city-validate
make render_snapshot      # offscreen screenshots (not part of `make`)
make test                 # all test groups (about 20 s)
make sanitize             # the same, under ASan and UBSan
make lint                 # cpplint
make fixture-city         # cook the small hand-written test district
make city EXTRACT=data/pordenone.osm      # cook a real city (about 40 s)
make new-city CITY=name BBOX="minlon minlat maxlon maxlat"   # fetch and cook
```

Run `python3 tests/run_tests.py citydrive` to run one group. In tests,
`malloc` and `calloc` are replaced by a leak-tracking allocator, so a leak
fails the test.

**CI is GitHub Actions on Ubuntu (gcc) and macOS (clang).** Ubuntu is the
stricter one, and it has caught real bugs that macOS hid: a missing
`#include <stddef.h>` for `size_t`, a struct read before it was set, a null
pointer passed to `qsort`. Lessons:
- Include what you use. Do not rely on a header pulling another in.
- Initialise every struct completely.
- `snprintf` into a small buffer of a longer string is deliberate in places;
  the test build passes `-Wno-format-truncation`.
- Before pushing anything that touches many files, reproduce the Linux job
  locally if Docker is available: copy the tree into an `ubuntu:24.04`
  container with `build-essential libsdl2-dev libgl1-mesa-dev python3
  cpplint clang-format` and run `make lint`, `make -j4`, `make test`,
  `make sanitize`, `make fixture-city` with `SDL_VIDEODRIVER=dummy` and
  `SDL_AUDIODRIVER=dummy`. Mounting the working tree directly leaves Linux
  object files that break the Mac build; copy it instead.
- After every push, check the Actions run for that commit and fix a red run
  before starting anything else. A red `main` is the top priority.

## 3. Looking at what you build

There is no human to play-test each slice, so use the snapshot tool.

```
./render_snapshot out.bmp SECONDS STEERING [package_dir|-] [overhead_height]
```

Environment variables set the scene: `SNAPSHOT_NO_DEBUG`,
`SNAPSHOT_CENTRE=x,z[,yaw]`, `SNAPSHOT_HOUR`, `SNAPSHOT_TRAFFIC`,
`SNAPSHOT_PEOPLE`, `SNAPSHOT_RAIN`, `SNAPSHOT_VEHICLE`, `SNAPSHOT_ORBIT`
(degrees round the car) with `SNAPSHOT_ORBIT_DISTANCE`, `SNAPSHOT_CRIME`,
`SNAPSHOT_FOOT`, `SNAPSHOT_ENTER`, `SNAPSHOT_SHOOT`, `SNAPSHOT_MAP`,
`SNAPSHOT_VIEW` (camera, and cockpit in an aircraft), `SNAPSHOT_FLY=n`
(take off in aircraft n), `SNAPSHOT_BAIL=t` and `SNAPSHOT_MENU`. Convert a
BMP with `sips -s format png` on macOS, then look at it. Add a variable
when a new feature needs a new scene. Keep temporary files outside the
repository.

Because nobody has played the game by hand, tuning numbers are guesses.
Expose them as named constants at the top of the file, say so in the commit
message, and prefer behaviour that is easy to adjust.

## 4. Where things are

```
engine/   core, math, platform (SDL window, input), renderer (+ gl), assets
          (city package format), world, physics, audio, vehicle, camera
game/src/ air, car, main, scenes, stage (the driving stage and its systems),
          textures, traffic (cars, people, fleet, navigation)
tools/    city_import, city_generate, city_validate, common (OSM reader,
          generator, projection, triangulation, LiDAR)
tests/    one test_*.c per area, render_snapshot.c, render_sound.c
docs/     architecture.md (read first), asset-pipeline.md, testing.md,
          geographic-data.md, vehicle-physics.md, decisions/
scripts/  fetch_osm.py, fetch_elevation.py, embed_shader.py
```

`docs/architecture.md` is the map: modules and the direction of
dependencies, the city generator, game systems, streaming, rendering, the
performance baseline, and a dated section for each pass of work. The
engine never depends on the game; the game never depends on the tools; the
tools share the city package format with the game and nothing else.

What exists today: real cities cooked from OpenStreetMap (Pordenone,
Barletta and others), streamed in chunks around the player; a vehicle
physics model with a fleet of cars, vans, buses, lorries and emergency
vehicles; traffic that follows lanes, obeys signals, gives way to people and
overtakes; pedestrians on pavements and crossings with fear, hit reactions,
punching and a thrown-body animation; a walking player with a pistol, a
punch, a car door and car theft that hauls the driver out; a wanted level
with police pursuit, WASTED and BUSTED resets; an airport with a light
plane, a helicopter, a Boeing 747, a jet fighter and a twin-rotor gunship,
cockpit and chase cameras, missiles, bailing out on a parachute; day and
night, rain, a radio, a procedurally synthesised soundscape, a minimap and a
full map, a satnav arrow, and a start menu.

Known gaps (good first slices): parked aircraft have no colliders, so cars
drive through them; a doomed aircraft has no smoke trail; car bodies have no
UV coordinates; the LiDAR importer has never been run on real data; there is
no glTF import; coastlines on islands are rough; people lack face and body
variety; there is no save game.

## 5. The roadmap

Each phase lists the player-facing goal, the slices, and how to know it is
done. Slices within a phase can be reordered if one unlocks another; phases
should be taken in order unless a later slice is a cheap win.

### Phase 0. A trustworthy base

Goal: every later phase builds on something that is fast, observable and
safe to change.

- **0.1 Green on both platforms.** CI passes on Ubuntu and macOS on every
  push, including `make sanitize`. Add a nightly job (or a manual
  workflow) that cooks a small fixture city and runs a headless drive for
  60 simulated seconds, failing on any assertion or sanitizer report.
- **0.2 A frame-time budget.** Add a `--bench` mode that flies a fixed
  camera path over a cooked city for 30 seconds and prints average and
  99th-percentile frame time, draw calls, triangles and physics time. Record
  the numbers for Pordenone in `docs/architecture.md` and fail a test when a
  later change makes them more than 25 per cent worse. Target 60 frames a
  second on an Apple M-series laptop and 30 on an integrated Intel GPU at
  1280 by 720.
- **0.3 Deterministic simulation.** Fixed timestep already exists; make the
  whole simulation (traffic, people, police, missions) a pure function of
  the seed and the input log. Add input recording and replay (`--record`,
  `--replay`) and a test that replays a short log and checks a state hash.
  Every bug report from now on can be a replay file.
- **0.4 Save and load.** A versioned plain-text save: position, vehicle,
  health, money, wanted level, mission state, time of day, weather,
  inventory, stats, and the seed. Autosave at safehouses and on mission
  completion; quick save and load keys. Round-trip test.
- **0.5 Settings and input.** An options screen: resolution, fullscreen,
  vsync, field of view, draw distance, shadow quality, volume sliders,
  invert Y, mouse sensitivity, dead zones. Key and button remapping stored in
  a config file. Full gamepad support with the on-screen help following the
  device in use.
- **0.6 Debug and developer tools.** An in-game console (backtick) with
  commands to teleport, spawn a vehicle or person, set time and weather, set
  the wanted level, give money, toggle collision shapes and navigation
  graph, and run a named mission. Used by the tests and by you.

### Phase 1. The player on foot

Goal: walking around feels like a third-person action game, not a walking
camera.

- **1.1 Movement.** Walk, jog, sprint with stamina, jump, crouch, climb low
  ledges and fences, swim (and drown), fall damage. Smooth acceleration,
  turning on the spot, a camera that never clips through walls (the chase
  camera already collides; extend it to the walking camera).
- **1.2 Health, armour and death.** Health bar, body armour, regeneration
  rules, hit reactions that depend on the weapon and direction, a short
  death camera, then hospital respawn (see 3.5 for the cost).
- **1.3 Weapons.** A weapon table in data, not code: fists, bat, knife,
  pistol, shotgun, SMG, assault rifle, sniper rifle, grenade, Molotov,
  rocket launcher. Per weapon: damage, rate, spread, magazine, reload time,
  range, noise, recoil, pickup model. Weapon wheel on a key or the pad's
  shoulder buttons. Ammo pickups and shops.
- **1.4 Aiming.** Free aim with a reticle, lock-on with a soft target, a
  first-person-ish zoom for the sniper rifle, aim-from-cover, aim-while-
  walking. Bullets are hitscan for small arms, projectiles for rockets and
  grenades. Surface impacts leave marks and spark; blood on people.
- **1.5 Cover.** Stick to a wall or car, blind fire, peek, step out. The
  physics world already gives you wall normals.
- **1.6 Animation.** Replace the box-and-limb figures with a small skeletal
  system: a skeleton, keyframed clips, blending, and procedural layers for
  aiming and looking. Generate the clips in code or from a tiny text format;
  do not import an animation library. Walk, run, jump, climb, swim, punch,
  kick, shoot, reload, enter and leave a car, die, get up.
- **1.7 Ragdoll.** Bodies hit by a car or an explosion become a simple
  articulated ragdoll on the existing physics world and settle on the
  ground. Bodies fade after a minute, out of sight only.
- **1.8 Character variety.** Faces, hair, skin tones, heights and builds,
  clothes and accessories per district, generated from a seed. Gangs wear
  their colours. The player can change clothes in shops (see 6.2).

### Phase 2. Vehicles

Goal: every vehicle is fun to drive and feels different.

- **2.1 Handling pass.** Per-vehicle handling files (mass, centre of mass,
  suspension, grip, power curve, gear ratios, drive layout, brake bias).
  Lateral-slip tyre model, handbrake turns, power slides, understeer and
  oversteer that can be felt. Tests for braking distances and for each
  vehicle staying upright on a straight, a bend and a kerb.
- **2.2 Damage.** Deformable body panels (move vertices of the chassis mesh
  near an impact), detaching bumpers, bonnets and doors, burst tyres, broken
  glass, steering that pulls, an engine that smokes, burns and explodes
  (exists in part). Damage affects handling.
- **2.3 More vehicles.** Motorbikes and scooters (lean, wheelies, rider on
  the bike), pickups, taxis, a limousine, a bus with passengers, a tram if
  the city has tracks, a train on real railway lines with level crossings,
  boats on real water (with a harbour and a marina), a sports car, a muscle
  car, a police interceptor, an armoured van, a tank (see 4.5). Each needs
  a mesh with UVs, handling, sounds, and a place it is likely to be found.
- **2.4 Vehicle interiors and entry.** Doors swing (done for the driver's
  door); add passenger doors, rear doors, a boot, and the animation of the
  player climbing in, from either side, with the nearest door chosen.
  Carjack variants: gentle (the driver steps out), rough (dragged out),
  and a threat with a gun at which the driver flees.
- **2.5 Driving extras.** Horn and siren, indicators, headlights with
  high beam, wipers, radio stations and a radio wheel (E cycles today),
  handbrake, nitro on some cars, a speedometer and rev counter on a dash
  that matches the cockpit camera, drive-by shooting from windows.
- **2.6 Garage and paint.** A Pay and Spray that resets the wanted level
  for a fee, a garage that stores cars between sessions (uses 0.4), and a
  modification shop (paint, wheels, power, armour) that costs money.
- **2.7 Aircraft and boats.** The airport exists. Add: colliders for parked
  aircraft, smoke and fire trails on a damaged aircraft, a landing-gear
  toggle, flaps, a believable airliner take-off and approach, a seaplane, a
  hot-air balloon, jet skis and speedboats with a wake, a lifeboat, and a
  ferry that moves between two quays on a schedule.

### Phase 3. The law

Goal: crime has consequences that escalate and can be escaped.

- **3.1 Witnesses.** A crime only counts if a person or an officer sees it
  (line of sight, distance, facing). Witnesses run to a phone or an officer
  and report; killing the witness stops the report. Crimes have weights.
- **3.2 Wanted levels one to six.** Each star has its own response:
  patrol cars, then more and faster ones, then roadblocks and spike strips,
  then SWAT vans and a police helicopter with a searchlight, then FBI-style
  agents in dark cars, then the army with tanks and helicopters at six.
  Responses spawn out of sight and are budgeted so frame time stays flat.
- **3.3 Police behaviour.** Officers on foot get out, shoot, chase and
  cuff; they call for backup; cars box the player in, use PIT manoeuvres and
  ram; helicopters track and shoot. They obey the same physics and
  navigation as everyone else (the nav graph exists in `navigation.c`).
- **3.4 Escaping.** The star flashes while the player is seen; hide out of
  sight for a time to lose them; Pay and Spray and changing clothes reset
  the level; bribes stars sit on the map as pickups.
- **3.5 Busted and wasted.** BUSTED loses weapons and a fine and starts at
  the police station; WASTED costs hospital fees and starts at a hospital.
  Both exist as resets today; give them a cost, a short cinematic and a
  lost-and-found for dropped weapons.
- **3.6 Other emergency services.** Ambulances that treat bodies, fire
  engines that put out burning cars, medics who revive the player in a
  mission. Siren behaviour makes traffic pull over (partly present).

### Phase 4. A living city

Goal: the street has its own life when the player is doing nothing.

- **4.1 Pedestrian brains.** A small behaviour layer (walk, wait, chat in
  pairs and groups, sit on benches, smoke, window shop, queue at a bus stop
  and board the bus, jog, push a pram, carry shopping, use a phone, fight,
  mug, flee, panic, call the police, film the player). Data-driven by
  district and time of day. Keep the per-person cost tiny and the density
  adaptive to the frame budget.
- **4.2 Traffic brains.** Road rage and horn use, accidents between AI cars
  that block a lane and call the emergency services, tow trucks, funeral
  processions, delivery stops, learner drivers, parking and unparking at the
  kerb, taxis that stop for a hail, buses on timetabled routes. Different
  traffic mixes by district and hour (commuter peak, night, market day).
- **4.3 Districts.** Classify areas of a real city from OpenStreetMap tags
  (old town, residential, industrial, commercial, park, harbour, airport,
  hospital, university, football ground) and give each its own building
  materials, signs, props, people, cars, music and crime rate. Show the
  district name when the player enters it.
- **4.4 Interiors.** Walk into selected buildings: a shop, a bar, a petrol
  station, a hospital, a police station, a safehouse, a tattoo parlour. A
  streamed interior cell with its own lighting and a door trigger on the
  facade. Start with box rooms from templates; a proper editor can wait.
- **4.5 The military.** An army base or airfield with a perimeter, guards,
  a tank and a hangar, which the player can infiltrate at five or six stars.
- **4.6 Ambient events.** Street robberies, a car chase between two AI
  vehicles, a drug deal in an alley, a fire, a protest, a street performer,
  a wedding with cars, a football crowd after a match, a funeral. Each is a
  small script spawned out of sight near the player at a low rate.
- **4.7 Animals and nature.** Birds that scatter, dogs on leads, cats, a
  few wild animals in parks, fish and ducks on water, trees that sway,
  grass that moves, a river that flows.
- **4.8 Weather and time.** Fog, storms with lightning and thunder, snow in
  winter, wet-road reflections (rain exists), puddles, wind that moves
  trees and flags, a sky with a sun, moon and stars at the right place for
  the city's latitude and the date, a season setting. People and traffic
  react to weather (umbrellas, slower driving, empty streets in a storm).

### Phase 5. Missions and story

Goal: the player is given things to do and a reason to care.

- **5.1 A mission framework.** Missions are data, not code: a small text
  format (or a tiny stack-based script) with stages, objectives, triggers,
  spawns, dialogue lines, checkpoints, fail conditions and rewards, parsed
  and validated by a tool and run by a `mission` module in `game/src/stage`.
  The objective HUD, the map marker for the current objective, the "mission
  passed" and "mission failed" screens, retry from the last checkpoint, and a
  replay list. Test with scripted input from 0.3.
- **5.2 Cutscenes.** Camera paths and actor scripts played in the engine
  with letterboxing and subtitles, skippable. No video files.
- **5.3 The phone and contacts.** A pause-menu phone: contacts who send
  messages and call with missions, a map, a notes list, a taxi service that
  drives the player to a destination, a wardrobe, a stats page.
- **5.4 Mission types.** Build the framework by writing these, one at a
  time, each with a real test: deliver a car, pick up and drop a person,
  chase and ram a target, follow without being seen, assassinate someone and
  escape, protect a convoy, rob a shop and flee, rob a bank with a getaway
  driver, escort, destroy a vehicle with a missile, steal a plane, a street
  race through checkpoints, a boat race, a drive-by, a rooftop sniper, a
  helicopter rescue, a prison break.
- **5.5 Story.** An original story arc of about a dozen missions in the
  generated city, with a small cast (a boss, a fixer, a rival, a cop), told
  in cutscenes and radio chatter. Keep the writing in a data file so it can
  be translated.
- **5.6 Side activities.** Taxi driver, vigilante (kill wanted criminals),
  paramedic, fire fighter, delivery rounds, stunt jumps (unique jumps found
  on the map), hidden packages, street races, shooting range, gambling.
  The old "delivery run" from the first prototype is gone; do not restore
  it as the main loop, make it one activity.
- **5.7 Progress.** A completion percentage, a stats screen, unlocks.

### Phase 6. Money and ownership

Goal: crime pays, and money buys something.

- **6.1 Cash.** Pick-ups, mission rewards, mugging, robbing shops, selling
  stolen cars at a yard, fines when busted, fees when wasted. A wallet
  and a bank. Cash drops from defeated enemies and floats as a pickup that
  vanishes after a minute.
- **6.2 Shops.** Clothes, weapons, food, a barber, a tattooist, a car
  dealer, a petrol station, a pharmacy for health. Opening hours. Each uses
  the interior system (4.4) or a street kiosk.
- **6.3 Property.** Buy safehouses (a save point and a garage), shops and
  businesses that pay a wage each in-game day, and defend them.
- **6.4 Needs and stats.** Health, fitness and stamina grow with use;
  driving, shooting and flying skills improve. Keep it simple and visible.

### Phase 7. A bigger, better world

Goal: more places, built with the same pipeline and no hand-modelling.

- **7.1 More cities.** Cook and ship notes for more real places with
  `make new-city`, each with a `PROVENANCE.md` and a short description for
  the menu: at least one large coastal city, one hilly city, one with a big
  river and bridges, and one with an island or two. Verify the validator
  reports zero issues for every one and add each to a smoke test that loads
  it and drives for ten seconds.
- **7.2 Coastlines, rivers and bridges.** Proper water meshes with shore
  blending, beaches, quays and piers, bridges and tunnels as continuous
  roads over and under the ground, harbours with cranes and containers.
- **7.3 Terrain and LiDAR.** Run the LiDAR importer on real tiles (it has
  never been run on real data), check the heights against the elevation
  model, and use it for rooftops, hills and accurate building heights.
- **7.4 Landmarks.** Recognisable buildings and monuments from OSM tags
  (castles, churches, stadiums, towers, bridges) with a few hand-tuned
  generators each, so each city has a skyline.
- **7.5 Streaming at scale.** Profile and fix the chunk loader so a
  20 km square city runs at the frame budget: level of detail for buildings
  and roads, impostors for far trees, instancing for repeated props,
  asynchronous chunk loading on a worker thread (write the decision record
  first), and a memory cap.
- **7.6 Navigation.** One shared navigation graph for pedestrians, cars,
  police, boats and aircraft routes, with a time-sliced path finder so a
  crowd never costs a frame.

### Phase 8. Look and sound

Goal: it looks and sounds like a game people would show to friends.

- **8.1 Models with UVs.** Give every car body UV coordinates and add a
  texture set per vehicle (paint, dirt, badges, tyre tread, glass). Better
  wheels, lights, mirrors and interiors.
- **8.2 Asset import.** A small, strict glTF 2.0 loader (meshes, materials,
  skins, animations) and a licence check in the asset pipeline, so
  artist-made, correctly licensed content can replace the procedural
  placeholders one at a time. No FBX, no engine.
- **8.3 Lighting.** Cascaded shadow maps, soft contact shadows, ambient
  occlusion, reflections on glass and wet roads from a cube probe, volumetric
  fog in the morning, headlight cones, lit windows at night with variety,
  neon signs, street lamps that flicker, lens flare, a light at every muzzle
  flash and explosion.
- **8.4 Post-processing.** Exposure adaption, bloom, depth of field in
  cutscenes, motion blur at speed (off by default), colour grading per time
  of day and per city, a film grain option, a heat haze over fire.
- **8.5 Particles.** A proper pooled particle system: smoke, fire, sparks,
  dust, tyre smoke, rain splashes, snow, leaves, water spray, debris,
  muzzle flashes, blood. One draw call per material.
- **8.6 Audio.** Positional 3D sound with occlusion and a reverb that
  follows the surroundings (street, tunnel, interior), a richer engine model
  per vehicle, tyre and surface sounds, weapons, impacts, footsteps by
  surface, crowd murmur by district, an in-world radio with original music
  and talk shows generated procedurally or licensed, and voice-over lines
  for pedestrians ("hey, watch it") through recorded or synthesised speech
  under a clear licence. Keep every ambient bed free of ringing pure tones.
- **8.7 User interface.** A consistent HUD (health, armour, weapon, ammo,
  money, wanted stars, minimap with blips, mission text, radio), a pause
  menu with map, stats, options, save and load, a start screen with a city
  picker that shows a thumbnail, subtitles, a font with accents for
  Italian, and a high-contrast mode.
- **8.8 Localisation.** All text in one table, English and Italian first,
  with a language option. Numbers, dates and distances follow the language.

### Phase 9. Polish, reach and longevity

- **9.1 Accessibility.** Subtitle size, colour-blind safe palette for the
  wanted stars and map, camera shake and flashing-light options, full
  remapping, hold versus toggle for every held key, an aim assist option.
- **9.2 Performance tiers.** Low, medium and high presets that change draw
  distance, shadow resolution, crowd density and effects, chosen from a
  quick benchmark on first run.
- **9.3 Packaging.** Reproducible release builds for macOS and Linux (and
  Windows if SDL2 and the build allow it without new dependencies), with
  the credits screen and licences bundled.
- **9.4 Modding.** Weapons, vehicles, missions, districts and cities as
  data directories that the game loads at start, with a validator tool and a
  short guide.
- **9.5 Optional multiplayer.** Only after everything above: a deterministic
  lock-step or a snapshot model, a decision record first, and a clear
  statement of what is out of scope (no matchmaking service, no accounts, no
  server the owner has to run).

## 6. How to choose the next slice

1. If `main` is red on any platform, fix that first.
2. If a standing behaviour in section 1 is broken, fix that next.
3. Otherwise take the lowest-numbered unfinished slice in the lowest-
   numbered unfinished phase, unless a later slice is under an hour and
   makes the game visibly more like GTA (a quick win).
4. After each slice, spend ten minutes on the player's view: render a few
   snapshots, look at them, and write down in `docs/architecture.md` what
   felt wrong and was not fixed.
5. Every fifth slice, do a refactoring-only slice: remove duplication, split
   a file over 800 lines, and delete dead code. Keep behaviour identical and
   the tests unchanged.

## 7. What to report

After each working session leave a short note in the commit message or in
`docs/architecture.md`: what the player can now do, what you tested and how,
what you looked at in snapshots, what you could not verify without playing
by hand, and the next slice you intend to take. Be plain about anything that
is a guess.
