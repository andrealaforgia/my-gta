# Asset pipeline

Source data is immutable; everything the runtime loads is produced by a
tool from source plus settings, and can be regenerated.

## Commands

```
city-import   --input <file.osm> --output <intermediate dir> [--no-service-roads]
city-generate --input <intermediate dir> --config assets/city/default.cfg --output <package dir> [--name <n>]
city-validate <package dir>
```

`make city EXTRACT=data/x.osm` runs all three. Exit codes: 0 ok, 1 failure,
2 bad arguments. Nothing overwrites source data.

## Intermediate model (ADR-005)

A directory with `manifest.txt`, `roads.txt`, `buildings.txt`. Lines are
records; `#` lines are comments.

```
road <osm way id> <class> <width m> <width inferred 0|1> <oneway> <layer> <bridge> <tunnel> <surface|-> <name|-> <point count>
p <x> <z> <osm node id>          one per point, local metres
junction <osm node id> <x> <z> <degree>
building <osm way id> <height m> <height source 0 tagged|1 levels|2 default> <levels> <category> <point count>
p <x> <z>                        anticlockwise seen from above, ring not repeated
```

The manifest records source path, import date, licence, geographic
bounds, projection origin and the import statistics (accepted, ignored,
degenerate, malformed, relations skipped, widths inferred, heights
defaulted).

## Cooked package

`manifest.txt` plus one `chunk_<column>_<row>.txt` per grid cell.

```
# city-package 1
name, source, source_date, licence, generated, generator_version, config_hash
bounds_lat_lon, origin, projection, chunk_size
spawn <x> <y> <z> <yaw>
chunk <column> <row> <file>
```

Each chunk file holds `bounds` and a list of parts:

```
part <material> <collider mesh|none> <layer> <friction> <source id> <vertex count> <index count>
v x y z nx ny nz u v
i a b c
```

Materials are names the game binds to its procedural textures: `road`,
`junction`, `pavement`, `kerb`, `grass`, `roof`, `facade0..3`. The game
merges all parts of one material in a chunk into one mesh, so draw calls
are about ten per chunk. Collider parts become static triangle meshes in
the physics world with the layer and friction given.

## Generation rules

- Roads become ribbons of their width, mitred at interior points, ends
  pulled back by the junction radius where a patch exists, at
  `road_height` above the ground.
- Junctions with degree three or more, and bends of more than
  `bend_patch_angle`, get a disc patch of the largest meeting half width
  times `junction_radius_scale`.
- Pavements of `pavement_width` at `kerb_height` run along both edges of
  every non-service road, with vertical kerb faces (render only). Pavement
  tops are colliders, so the car can mount them; kerbs never trap wheels.
- Buildings are extruded footprints: wall quads with outward normals and a
  texture repeat per `window_width` and `storey_height`, an ear-clipped
  roof. Facade variant is a stable hash of the source id. Both are
  colliders on the building layer.
- Ground tiles cover every chunk cell at y = 0 with grass.
- The spawn is the midpoint of the longest segment of the longest
  non-service road, facing along it.
- Output is deterministic for the same model and settings; `config_hash`
  in the manifest is the settings hash.

## Validation

`city-validate` checks every part has vertices and triangles, indices in
range, finite positions, unit normals and no degenerate triangles; that
vertices lie inside their chunk bounds; that the spawn is inside a chunk;
that there are road surfaces and colliders; and that source and licence
are recorded. It prints counts and exits 1 on any issue.
