# Phase 0.1: scheduled headless city drive

## DISCUSS

The roadmap requires a nightly or manually triggered fixture-city drive for
at least 60 simulated seconds that fails on assertions and sanitizer reports.
The existing citydrive group already cooks the hand-written district and
drives a city with 20 traffic cars and 40 pedestrians for 68 simulated seconds.
Reuse it rather than introduce a second simulation harness.

## DESIGN

Add `make soak` to run the citydrive group under ASan and UBSan through the
existing runner and stub renderer. A separate scheduled/manual workflow runs
this command on Ubuntu and macOS with dummy SDL drivers and a job timeout.
The runner already propagates assertions, non-zero exits, timeouts, tracked
allocation leaks and sanitizer errors. No runtime module or dependency changes.

## DISTILL

The CLI acceptance check is `make soak`: it must execute the sanitizer-backed
citydrive group successfully without a window or a GPU. Before implementation
it failed with `No rule to make target 'soak'` (exit 2).

Strengthen the existing minute-long drive to check all three position axes,
heading and speed for traffic and pedestrians, and prove the player actually
moves. Keep the full citydrive group in the ordinary test and sanitizer runs.

## Slice plan

| Slice | Value | Status |
|---|---|---|
| slice-01 | Run the busy fixture city under sanitizers nightly and on demand | shipped |

## DEVOPS

Validate lint, build, tests, sanitizers and fixture cooking on macOS and in
an isolated Ubuntu container. Commit and push the green slice to main, inspect
the push workflow and manually dispatch the soak workflow. GPU rendering is
unchanged; record a fixture snapshot as a separate visual check.

## Review and validation

Independent plan and implementation reviews approved this slice without
blocking findings. `make soak` changed from a missing-target failure to a
passing sanitizer run. The clean macOS copy passed lint, build, all 19 test
groups, all 19 sanitizer groups, soak and fixture cooking/validation.

The [push workflow](https://github.com/andrealaforgia/my-gta/actions/runs/40101971343)
and the manually dispatched
[soak workflow](https://github.com/andrealaforgia/my-gta/actions/runs/40102009952)
both passed on Ubuntu and macOS at commit `24eff3b`. A concurrent aircraft
change committed the staged soak implementation in that commit; history was
preserved. The final session note is a separate commit.

Viewed `/tmp/gta-like-soak-fixture.png`: the three-second fixture drive
renders streets, cars, pedestrians and HUD. The bottom control hint has low
contrast against pale ground; retained as a future UI issue. Handling still
needs manual play-testing. Next slice: roadmap 0.2, frame-time benchmark.

The fixture uses fixed `/tmp` directories, so same-host citydrive runs must
remain sequential. No production runtime code changed in this slice.
