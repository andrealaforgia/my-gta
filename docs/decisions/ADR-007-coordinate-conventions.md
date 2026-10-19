# ADR-007: Coordinate and transform conventions

Status: accepted, 2026-10-20

- Right-handed. +X east, +Y up, +Z south. Forward for a vehicle at identity
  orientation is -Z (the OpenGL camera convention), so "north" in the city
  is -Z.
- Metres, seconds, radians, kilograms.
- Matrices are 4x4, column-major, `m[column * 4 + row]`, so they upload to
  GL without transposing. Vectors are columns: `v' = M * v`.
- Quaternions `(x, y, z, w)`, unit length, rotate vectors as `q v q*`.
- Triangles are counter-clockwise when seen from outside; back faces culled.
- Depth range is the GL default; the projection is a standard perspective
  with vertical field of view.
- Physics steps at 1/120 s; rendering interpolates between the last two
  physics states.
- Geographic input is reprojected in the importer to this world frame:
  local x = east metres, local z = -north metres. No axis swap exists
  anywhere else.

Tested in `tests/test_math.c` and `tests/test_city.c`.
