# Phase 0.2: a repeatable frame-time budget

## DISCUSS

The next roadmap slice needs a moving-city benchmark so performance changes
can be measured against the same workload. The snapshot tool currently times
a stationary view. Run the real game with `--bench` and a cooked city, render
a fixed route, and report average and 99th-percentile completed-frame time,
draw calls, triangles and simulation time at 1280 by 720 with vsync disabled.
Keep the default workload independent of machine speed: 1800 frames, each
with two 1/120 s simulation steps, represent 30 simulated seconds.

## DESIGN

Keep the runner and summary/budget helpers under `game/src/main/benchmark`.
The game CLI bypasses the menu and audio for benchmark runs. Reuse the actual
driving stage, scene streaming, traffic and pedestrians, overriding the
camera and its streaming focus along the deterministic route. Exclude loading
and initial rendering warm-up from the samples. Wait for submitted rendering
to finish before recording a frame; otherwise the number measures CPU
submission, not completed rendering.

Use the existing line-oriented config reader for an optional baseline file.
Validate all required metrics and resolution. Bind the baseline to a stable
workload identity including the city/config/bounds, route and sample count,
actor counts, time/weather and rendering settings. Refuse unlike workloads
rather than report a misleading comparison. Compare only when a baseline
is explicitly supplied: timings from unlike GPUs cannot fairly share a gate.
At exactly 125% of baseline pass; above it return a failing exit status.
Record the actual Pordenone baseline and machine details after GPU execution.
The simulation metric includes the stage step and streaming, not just the
physics solver. No dependency or language change.

## DISTILL

Tests exercise benchmark summary and nearest-rank p99, the exact regression
boundary, malformed/nonfinite/missing baseline values, resolution mismatch,
the deterministic moving camera path and the runner through the stub
renderer. A short test workload reaches actual camera/render/simulation
wiring without running 1800 frames in every sanitizer test. Verify the built
CLI rejects missing cities and invalid benchmark options, writes a baseline,
and returns failure for an intentionally strict baseline.

## Slice plan

| Slice | Value | Status |
|---|---|---|
| slice-01 | Measure the same city route and detect a 25% regression | implemented; hosted CI gate pending |

## DEVOPS

Run lint, build, the full test suite, sanitizers and fixture cooking. Run the
real GPU benchmark on Pordenone, inspect its camera view and save the machine
baseline. Verify failure with an artificially low budget. Independently
review the feature before integration. Work in an isolated worktree while
other work is active; preserve concurrent edits and integrate green code
onto main in a small commit, then check macOS and Linux CI.

## DELIVER and review

The first summary assertion failed against inert stubs. Subsequent RED
checks exposed the missing CLI, camera aim, shadow-mode identity and invalid
numeric CLI handling. All five C cases and four Python CLI cases now pass.
The runner acceptance checks actual simulation advancement, final camera and
renderer projection, plus a strict budget failure after measured frames.
An independent review found and corrected malformed-baseline tests that had
previously failed on record count before reaching numeric validation. A
temporary mutation disabling numeric validation was then killed by the
corrected tests. Independent final review approved the scoped implementation.

The isolated macOS tree passed lint, build, all 20 test groups, all 20
sanitizer groups, CLI acceptance and fixture cooking/validation. Ubuntu 24.04
with GCC passed lint, build and CLI acceptance. Its local Docker sanitizer
check hit the unchanged 60 s group timeout: a separate diagnostic showed
more than 207 s inside existing stage initialisation, before benchmark_run
was reached. No source or timeout was weakened to accommodate the VM;
hosted Linux CI remains the cross-platform sanitizer gate.

The actual Pordenone GPU command wrote the reference under
`docs/baselines/pordenone-m4-pro.cfg`. A repeat passed the baseline: mean
12.869 ms, p99 17.642 ms, simulation 0.813 ms, with exactly the same mean
draw and triangle counts. An intentionally tiny frame-time baseline made
the built CLI return exit 1 after all 1800 measured frames, proving CLI
budget wiring. Actual camera captures at the start and quarter orbit were
inspected; the route now looks down at the city rather than mostly sky.
See the dated architecture section for the measured reference and limits.
