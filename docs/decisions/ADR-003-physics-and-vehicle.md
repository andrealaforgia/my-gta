# ADR-003: Physics and vehicle dynamics

Status: accepted, 2026-10-20

## Decision

A narrow in-house physics module: one class of dynamic body (the car chassis,
a box with mass and inertia), static colliders (triangle meshes for roads and
ground, boxes for buildings and barriers), ray casts, and a fixed-step
integrator. The car is a rigid chassis on four raycast wheels with
spring-damper suspension and a slip-based tyre model.

## Why

Jolt is C++ with no official C API. Wrapping it means a C++ toolchain and a
large dependency for a prototype with one dynamic body. The spec allows "a
narrow raycast-wheel vehicle controller on top of rigid body and collision
queries"; that is what is built, and the rigid body underneath it is the
only one the prototype needs.

## Limits accepted

No dynamic-versus-dynamic collision, no joints, no sleeping, no broadphase
beyond chunk bounds. Chassis-versus-static contact is resolved at the
chassis box corners against boxes and against the ground mesh. These are
documented in `docs/vehicle-physics.md` and revisited only when the
prototype needs more.
