# gta-like

A custom 3D engine in C99 and SDL2, and a free-roaming driving prototype
on top of it: a car with raycast-wheel suspension and tyre grip on a test
track, or in a small city generated offline from real OpenStreetMap data.
Everything is synthesised or procedural; there are no image or audio
assets. Built from the plan in `docs/` (see the decision records) with the
owner's constants: C and SDL2.

## Setup

```
make install        # SDL2 via Homebrew (or apt on Linux)
make dev_install    # cpplint and clang-format (optional)
```

macOS on Apple silicon is the reference platform. Rendering is OpenGL 4.1
core through SDL2 (see `docs/decisions/ADR-002-rendering-api.md`).

## Build, run, test

```
make                # the game and the three city tools
make run            # drive on the test track
make test           # every test group, no window needed
make sanitize       # the same under AddressSanitizer and UBSan
make lint
make snapshot       # render a frame of the track to snapshot.bmp (needs a GPU)
./gta-like --help
```

## Generate and drive a real city

```
# 1. Get a small OSM XML extract (ODbL, attribution required). For example
#    central Pordenone, about 780 m square:
curl -A "gta-like-prototype/0.1" -o data/pordenone.osm \
  "https://api.openstreetmap.org/api/0.6/map?bbox=12.6540,45.9545,12.6640,45.9615"

# 2. Import, generate and validate (writes assets_cooked/cities/pordenone)
make city EXTRACT=data/pordenone.osm

# 3. Drive it
make run-city CITY=pordenone
```

`make fixture-city` cooks the small hand-written district in
`tests/fixtures/district.osm` instead, for a city that needs no download.
See `docs/geographic-data.md` and `docs/asset-pipeline.md`.

## Controls

Arrows or WASD drive and steer, space handbrake, R reset to the spawn,
P pause, C re-centre the camera, F1 debug overlay, M mute, F11 fullscreen,
Esc quit. A game controller steers with the left stick, accelerates and
brakes with the triggers, A handbrake, Y reset, Start pause. Holding the
brake at a standstill engages reverse; the throttle brings you back.

## Layout

```
engine/   core math platform renderer(gl) assets world physics audio vehicle camera
game/     the prototype: scenes, car visuals, stage, HUD, main
tools/    city-import, city-generate, city-validate and their shared code
tests/    one binary with named groups, run by tests/run_tests.py
assets/   vehicle and generator settings, shaders live in the renderer
data/     raw extracts (not committed) and their provenance notes
docs/     architecture, decisions, pipeline, physics, testing
```

## Status

What works: the engine (fixed 120 Hz physics with render interpolation,
forward renderer with shadows, fog and debug drawing, synthesised audio), a car
that accelerates, brakes, steers, reverses, skids and collides, a chase
camera, the offline pipeline from OSM XML to a validated cooked package,
and driving through the generated city: roads, pedestrian streets and
paths, squares, parks with trees, car parks, water, lamp posts, and
buildings with courtyards, plaster facades and terracotta roofs.
`docs/testing.md` lists what the tests cover and
`docs/vehicle-physics.md` the handling model and its limits. Not done:
glTF import, elevation, chunk streaming, lane markings at junctions. See the end of
`docs/architecture.md` for the measured performance baseline.
