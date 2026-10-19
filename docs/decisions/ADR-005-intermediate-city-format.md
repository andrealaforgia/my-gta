# ADR-005: Intermediate city format

Status: accepted, 2026-10-20

## Decision

A versioned line-oriented text format, one record per line, written by
`city-import` and read by `city-generate`. The cooked city package uses the
same style: a manifest, chunk files with mesh and collision records.

## Why

Debuggable with `grep` and `diff`, trivial to parse in C without a JSON
library, deterministic to write. Binary output can be added later if volume
demands it; the readers are isolated in `tools/` and `engine/assets/`.

## Format

See `docs/geographic-data.md` and `docs/asset-pipeline.md`.
