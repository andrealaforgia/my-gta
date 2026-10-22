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
make sound          # render an engine run to engine.wav to judge by ear
./gta-like --help
```

## Generate and drive a real city

```
# 1. Get an OSM XML extract (ODbL, attribution required). A small one comes
#    straight from the API, for example central Pordenone, 780 m square:
curl -A "gta-like-prototype/0.1" -o data/pordenone.osm \
  "https://api.openstreetmap.org/api/0.6/map?bbox=12.6540,45.9545,12.6640,45.9615"
#    Pordenone with Cordenons (5 km square, streamed as you drive) is fetched
#    in tiles; also fetch its elevations for the hills:
python3 scripts/fetch_osm.py 12.648 45.949 12.712 45.992 data/pordenone.osm
python3 scripts/fetch_elevation.py 45.949 12.648 45.992 12.712 \
  data/pordenone.elevation.txt 40

# 2. Import, generate and validate (writes assets_cooked/cities/pordenone)
make city EXTRACT=data/pordenone.osm

# 3. Drive it
make run-city CITY=pordenone
```

Barletta is fetched the same way (`python3 scripts/fetch_osm.py 16.262 41.306
16.306 41.336 data/barletta.osm`, then `make city EXTRACT=data/barletta.osm`).
Run `./gta-like` or `make play` with no scene and a menu lists the test track
and every cooked city under `assets_cooked/cities`; `--city=barletta` goes
straight to one.

`make fixture-city` cooks the small hand-written district in
`tests/fixtures/district.osm` instead, for a city that needs no download.
See `docs/geographic-data.md` and `docs/asset-pipeline.md`.

## Controls

| Key | Does |
|---|---|
| arrows or WASD | drive and steer (holding the brake at a standstill reverses) |
| space | handbrake |
| R | reset to the spawn |
| C | change view: chase, bonnet, cockpit |
| H | horn (hold) |
| V | swap to the next vehicle: four cars, police car, ambulance, fire engine, bus and six lorries |
| M | the city map over a paused game, with street names and your position |
| P | pause |
| T | rain on or off |
| N | mute |
| F1 | debug overlay |
| F11 | fullscreen |
| Backspace | back to the start menu, to choose another city |
| Esc | quit (in the start menu too; arrows or W/S and Enter choose) |

A game controller steers with the left stick, accelerates and brakes with
the triggers, A handbrake, Y reset, Start pause.

Command line: `--time=H` starts the clock at hour H (default 10), `--hour-seconds=S`
sets real seconds per game hour (default 30, 0 freezes it), `--traffic=N` and
`--pedestrians=N` set how many cars and people there are (70 and 180), `--rain` starts in the rain.

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
forward renderer with shadows, fog, a sky with sun, moon and stars, emissive
lamps and windows, a headlight cone, synthesised audio), a car that
accelerates, brakes, steers, reverses, skids and leaves marks, collides and
leans like a real one, four cars to drive, three views, the offline pipeline from OpenStreetMap
XML to a validated cooked package, and driving through the generated city:
real streets with stop lines, zebra crossings, stop signs and manhole covers,
hills from an elevation model, a day that runs from dawn to night with lit
windows and street lamps, parked and moving cars that follow the roads and
give way (buses, lorries, police cars, ambulances and fire engines with sirens among them), people walking the pavements and crossing at zebras, lamp posts, signals and signs that fall when hit, shop fronts, porticoes with arches and
painted facades along the Corso, a brick town hall with a clock tower,
campanili, parks, water and trees, benches, bus stops, traffic lights at busy junctions that cars
obey, a minimap in the corner and the name of the street you are on, delivery runs (a pillar of light on a distant road, an arrow, a clock and cash to earn), the sea along a coast, drivers who sound their horns when you hold them up, rain with wet roads, and the church bell on the hour.
`docs/testing.md` lists what the tests cover and
`docs/vehicle-physics.md` the handling model and its limits. Not done:
glTF import, bridges for cars. See the end of
`docs/architecture.md` for the measured performance baseline.
