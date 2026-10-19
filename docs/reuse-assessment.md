# Reuse assessment: sdl-2d-game-engine and car-driving

Reviewed on 2026-10-20 against `~/dev/games/car-driving` (engine submodule at
the commit pinned there).

## Reused as ideas, rewritten for 3D

| Module | What carries over | What does not |
|---|---|---|
| `driving_stage.c` fixed-step loop | Accumulator, catch-up cap, simulation separate from render. | Tied to the stage struct; here it becomes a standalone, unit-tested `frame_clock`. |
| `command_line.c` | Flag parsing style, `--help`, validated numeric options. | Options are the 2D game's; rewritten with the 3D set. |
| `logger.h` | Severity macros over SDL logging. | Adds a subsystem label. |
| `car.c` | Torque curve, seven gears, drag, engine braking, grip that limits lateral speed change. | 1D track position and normalised lateral x. The 3D car is a rigid body on four raycast wheels. |
| `engine_synth.c` / `engine_note.c` | Synthesised exhaust pulse train driven by revs and throttle, so no audio samples and no licences. | Mixer music hook; here it feeds a plain SDL audio callback. |
| `tests/run_tests.py` + `test_allocator` | One test binary, named groups, allocation tracking, sanitiser run, no window. | None. Copied in shape. |
| `object_pool.c` | Could serve entity storage later. | Not needed for the first slice; left out until a use appears. |

## 2D-specific, left behind

`graphics*.c`, `texture.c`, `drawing_primitives.c`, `text.c`, `ttf_text.c`,
`bitmap_font.c`, `arcade_font.c`, `frame.c`, `render_utils.c`, `coords.h`
all sit on `SDL_Renderer` and 2D points. The 3D path goes through an OpenGL
context, so none of it applies. `geometry.c`, `physics.c`, `collision.c` are
2D point and circle maths. `keyboard.c` hard-codes the key set in its API;
the 3D input layer maps actions to keys and controller axes instead.

## Verdict

The engine's value is its working practices (headless tests, sanitisers,
small modules, one binary per concern), not its graphics code. The 3D engine
is a fresh C99 tree that keeps those practices and ports the vehicle and
audio ideas from the game.
