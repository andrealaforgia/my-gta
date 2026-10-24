# Testing

`make test` compiles every engine, game and tool source (except `main.c`
files and the OpenGL backend) with a tracking allocator into one binary
and runs each group; a group fails on an assertion, a non-zero exit, a
timeout or a leaked allocation. `make sanitize` repeats it under
AddressSanitizer and UndefinedBehaviorSanitizer. A stub renderer
(`tests/test_renderer_stub.c`) stands in for OpenGL, so nothing here
needs a window or a GPU.

`make soak` runs the citydrive group under ASan and UBSan, including a busy
fixture city driven for 68 simulated seconds with 20 traffic cars and 40
pedestrians. It checks that the player moves, stays above ground, and that
traffic and pedestrian positions, headings and speeds remain finite. The
tracking allocator also checks cleanup. No downloaded city or GPU is needed.
The `Headless city soak` workflow runs this on Ubuntu and macOS daily at
03:23 UTC, or on demand through GitHub Actions' Run workflow button. The
ordinary push workflow continues to run this group in both full test suites.

| Group | Covers |
|---|---|
| math | vector, quaternion, matrix, transform, box, ray, plane, frustum and damping behaviour under the ADR-007 conventions |
| core | fixed-step frame clock and catch-up cap, configuration parsing and defaults, keyboard mapping, dead zones, steering filter |
| citymap | the pause-screen map: roads and roofs painted north up, points off the map |
| daylight | the sun's arc, sky, stars, lamp glow, no jumps over a day, the bell strikes |
| traffic | fleet, parked cars, lanes, junction right of way, stopping, people on pavements, skid marks, cars following corners without jumps, drivers who honk, the satnav (one-ways, islands), police pursuit, the law (stars, bust, getaway), the player on foot |
| textures | road wear and markings |
| mesh | box and cylinder builders, winding, appending, subdividing, the 5x7 font |
| physics | the collider grid (what is near, moves, removals, slot reuse), a dropped box lands and rests, contact impulses, a wall stops a slide, friction, ray casts with layers and friction, torque from point forces, interpolation state |
| vehicle | parameter loading, settling, 0 to 50 km/h, braking distance, turning direction, reverse, wet grip, reset, wheel transforms, handbrake skid |
| camera | settles behind and above, follows at speed, a wall pulls it in, reversing |
| audio | audible at idle, pitch follows revs, throttle raises level, tonality at the firing period, silence when off, skid and impact voices, distance attenuation |
| world | stable ids and bounds, frustum visiting, drawing only what is visible |
| osm | the fixture loads, bad files are refused, entities unescape, projection to metres |
| city | road classification and splitting, cleaning, junction degrees, building heights and winding, deterministic defaults, text round trip |
| junction | corner intersections, cuts, exact patches, bevels, forks, pavement corners, polyline cutting |
| generator | ear clipping, every feature becomes geometry, widths and heights, corridor fitting, lamp posts, winding, spawn on a road, package round trip and validation, determinism, settings hash |
| citydrive | the fixture city loads from a cooked package, the car drives its spawn road and through a junction, a building stops it |

`make sound` renders ten seconds of a standing start (idle, a skid at
launch, full throttle through the gears with wind rising, an impact,
lift-off) to `engine.wav` with no audio device, so the engine can be
judged by ear. The audio tests bound the engine's tonality at the firing
period, so the note stays a note and not a hiss.

Long checks run through the snapshot tool: with `SNAPSHOT_TRAFFIC=N`,
`SNAPSHOT_STUCK=1` reports cars that stood still for 40 s or more,
`SNAPSHOT_JERK=1` cars that jump or spin between steps,
`SNAPSHOT_CRIME=1` starts a police chase and reports it,
`SNAPSHOT_SOAK=1` drives about at random getting out, stealing cars and
changing vehicle and fails on a NaN, and `SNAPSHOT_TRACE=file` writes every
car's path for plotting. Run `SNAPSHOT_STUCK=1` after any change to traffic.

GPU checks are separate: `make snapshot` renders the track through a
hidden window and fails if the frame is blank; `./render_snapshot out.bmp
<seconds> <steering> <package dir>` does the same in a city and prints
draw calls, triangles and milliseconds per frame.

Manual checklist before calling a milestone done: launch from a clean
build, resize the window, drive onto the pavement and into a building,
press R after flipping the car, F1 for the overlay, M to mute, quit with
Esc and check no audio keeps playing.
