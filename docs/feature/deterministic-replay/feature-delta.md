# Phase 0.3: record and reproduce a game session

## DISCUSS

A bug report should include a small input log and seed that reproduces the
session. Add `--record=<file>`, `--replay=<file>` and a reproducible `--seed`.
Replay should reject incompatible initial conditions and identify the first
frame where state differs, rather than silently continue with a different
game. Guarantee the same build, scene data and platform; cross-architecture
floating-point identity is outside this slice.

## DESIGN

The current stage combines fixed 1/120 s physics with frame-duration-driven
weather, clock and camera updates. Record the exact frame duration as part
of each external input, along with driving controls, horn and game actions.
Replay the same sequence through the real frame dispatcher and advance
service, irrespective of playback wall-clock speed. This preserves current
long-frame and weather semantics without maintaining a second game loop.

Use strict versioned plain text, lossless numeric representations, an initial
context/state fingerprint and per-frame state hashes, plus a final record.
Write and flush each complete input record before applying it, then append
a separate post-state checksum. A crash inside the simulation must leave
the input that caused it available for replay. Flush the post-state before
rendering as well, preserving verified frames if rendering crashes. Replay
applies a valid pending input before requiring its checksum; if execution
returns, a missing checksum
is an explicit incomplete-session failure, never a successful replay.
Persist the effective seed, initial time/rate, rain, actor counts and vehicle
identity in the header. Replay restores those settings and rejects explicit
conflicting overrides; a bug report must not require guessing its old CLI.
Reject malformed/nonfinite fields, unknown versions/actions, truncation,
trailing records and mismatching context. Hash observable gameplay state,
never pointers, padding, renderer handles, audio phases or measured timings.
Include the vehicle/player, law, time/weather, traffic and pedestrians,
aircraft/projectiles/parachute and persistent collision/world changes.

The stage owns the replay services; main only composes CLI options and stage
lifecycle. Expose a seeded initialisation path while preserving the existing
initialisation API and defaults. Traffic uses the chosen seed; pedestrians
derive theirs from it. Baseline identity must account for the new seed knob.
Record one scene per file; a menu return must not overwrite that recording.
Avoid persisting personal or absolute configuration paths in the log.
City packages and custom vehicle files remain matching external assets;
their identity must be checked before replay begins.

## DISTILL

Acceptance exercises actual recording and replay through stage services,
including changing controls, actions, pause, long frames and weather. Repeated
replay must reproduce frame hashes; changing input or seed must be detected.
Tests cover strict round trips and malformed/truncated logs. They also cover
a complete final input without its state record: it is
applied, then reported as unverified so the crashing step can be reproduced.
CLI checks cover incompatible options and numeric validation. `--record-frames`
allows a bounded real GPU recording/replay smoke check without manual input.
Use the actual stage initialisation and preserve the normal test timeout.

## Slice plan

| Slice | Value | Status |
|---|---|---|
| slice-01 | Reproduce an interactive session from seed and checked frame log | shipped |

## DEVOPS

Run lint, build, full native tests/sanitizers and CLI checks. Run a bounded
recording, replay it through the built game, and verify a tampered log fails.
Recheck and refresh the GPU baseline identity after introducing the seed.
Review independently, integrate a small green commit onto main and check
both hosted platforms before moving on.

## DELIVER and review

RED began with actual-stage header recording against inert stubs; separate
CLI checks failed against the missing options. Independent review required
direct semantic assertions so record and replay could not agree on ignored
actions. Tests now independently verify pause/map freezing, long-frame
physics capping with full-duration weather/time, representative actions,
seed propagation, state-family sensitivity and renderer/timing exclusions.
Tampering a real third frame fails after two verified frames.

Review also found a blocking crash-recording flaw: applying a frame before
writing it lost the crashing input. A RED pending-input test drove the final
two-phase flushed input/state format. It now executes an intact pending
input and reports it unverified, distinguishing that from a verified prefix
without a footer. Quit-event testing exercises the actual SDL event queue
and shared live loop, proving exactly one complete session. Final independent
review approved the implementation. Production-only refactoring removed
duplicate vehicle/content fingerprint implementations from the benchmark.

Validation passed: native lint/build, all 21 C groups, all 21 sanitizer groups,
eight CLI test methods, fixture cooking/validation; Ubuntu GCC lint/build,
the eight CLI methods, replay/benchmark C groups and fixture validation.
The known local Docker ASan stage-init slowdown was not worked around;
hosted Linux remains the full sanitizer gate.

Actual built-game GPU checks passed: a 24-frame track recording restored
non-default seed/time/weather settings at a different playback resolution;
tampering was rejected at frame 3; a pending third input ran and was reported
unverified. A 12-frame Pordenone recording replayed with seed 71, rainy 19:00,
hour rate 7, 12 cars and 20 pedestrians restored from its header.

The new benchmark identity includes seed and shared scene/vehicle content.
Its refreshed reference measured 13.052 ms mean frame, 17.925 ms p99 and
0.841 ms simulation. Against the preceding reference these changed by
+0.56%, -7.01% and +8.58%; draw and triangle counts were identical. All five
metrics passed the old 25% gate before the reference was replaced.

Viewed `/tmp/gta-like-replay-rain.png` on Airport Road: traffic, rain, lamps,
headlights, wet road and HUD remain visible. Dark verges and the low-contrast
bottom control hint remain. Handling/camera feel and Intel GPU performance
still require separate checks. Next is roadmap 0.4, save and load.

The integrated commit `fee16f9` passed the complete
[macOS and Linux workflow](https://github.com/andrealaforgia/my-gta/actions/runs/40110960810),
including both full sanitizer suites and all CLI checks. Concurrent map/HUD
changes were preserved and the integrated native suites were rerun. The
rebuilt main executable replayed the pre-merge Pordenone log and passed its
GPU baseline (12.984 ms mean, 18.193 ms p99, 0.797 ms simulation; unchanged
draw and triangle counts). Temporary implementation worktrees and the test
container were removed. The working tree was clean after integration.
