# ADR-001: Language, build and project structure

Status: accepted, 2026-10-20

## Decision

C99, as in the existing 2D engine. GNU Make and a Python test runner, as in
the existing projects. Not C++20, not CMake.

## Why

The owner set C and SDL2 as the constants. The spec preferred C++20 for Jolt
interop; without Jolt (ADR-003) that reason goes away. Make already builds
the 2D engine and the car game, and CMake is not installed on the reference
Mac. The spec's build requirements (debug and release flags, warnings as
errors, sanitiser tests, clean-checkout build, generated files ignored) are
met by the Makefile and `tests/run_tests.py`.

## Layout

```
engine/   core math platform renderer(gl) assets world physics audio vehicle camera
game/     the driving prototype only
tools/    city_import city_generate city_validate (offline, no window)
tests/    one binary, named groups, allocation tracking
assets/   shaders, vehicle parameters, scenes, small fixtures
data/     raw geographic extracts, not committed
```

Dependency direction is one way: `core` and `math` at the bottom; `platform`,
`renderer`, `physics`, `audio`, `world` above them; `vehicle` and `camera`
above those; `game` composes everything; `tools` use `math` and `core` only.

## Consequences

The 3D engine is a separate tree from the 2D engine, not a fork of it. It can
be split into its own repository later, as the 2D engine was.
