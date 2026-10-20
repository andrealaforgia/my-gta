# Asset pipeline

Source data is immutable; everything the runtime loads is produced by a
tool from source plus settings, and can be regenerated.

## Commands

```
city-import   --input <file.osm> --output <intermediate dir> [--service-roads]
city-generate --input <intermediate dir> --config assets/city/default.cfg --output <package dir> [--name <n>]
city-validate <package dir>
```

`make city EXTRACT=data/x.osm` runs all three. Exit codes: 0 ok, 1 failure,
2 bad arguments. Nothing overwrites source data.

## Intermediate model (ADR-005)

A directory with `manifest.txt`, `roads.txt`, `buildings.txt`, `areas.txt`. Lines are
records; `#` lines are comments.

```
road <osm way id> <class> <width m> <width inferred 0|1> <oneway> <layer> <bridge> <tunnel> <surface|-> <name|-> <point count>
p <x> <z> <osm node id>          one per point, local metres
junction <osm node id> <x> <z> <degree>
building <osm id> <height m> <height source 0 tagged|1 levels|2 default> <levels> <category> <point count>
p <x> <z>                        anticlockwise seen from above, ring not repeated
area <osm id> <water|park|parking|square> <point count>
p <x> <z>
```

Road classes are motorway, primary, secondary, tertiary, residential,
service, unclassified, pedestrian, path and waterway.

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
`junction`, `pavement`, `kerb`, `grass`, `roof`, `facade0..3`, `lamp`,
`paving` (pedestrian streets, paths, squares), `parking`, `park`, `water`. The game
merges all parts of one material in a chunk into one mesh, so draw calls
are about ten per chunk. Collider parts become static triangle meshes in
the physics world with the layer and friction given.

## Generation rules

- Each road's width is fitted to its surroundings first: the nearest
  building edge to the centre line sets the corridor, and the road and
  its pavements shrink to fit (road half width never below 1.75 m,
  pavements dropped below 0.4 m). Unconstrained roads keep the imported
  width.
- At every junction (`tools/common/junction_geometry.c`) the adjacent
  road edges are intersected to find the corner points; each road is cut
  back to where both its edges clear the corners plus a 0.2 m margin,
  and the gap is filled with the exact polygon through the corners and
  road ends. Sharp corners are bevelled instead of mitred. A straight
  continuation needs no patch; a bend of any angle gets one.
- Roads become ribbons of their fitted width on the cut polyline,
  mitred at interior points, at `road_height` above the ground.
- Pavements of the fitted width at `kerb_height` run along both edges of
  every non-service road, with vertical kerb faces (render only), and
  pavement corner fills with kerbs join neighbouring arms around each
  junction. Pavement tops are colliders, so the car can mount them;
  kerbs never trap wheels.
- Pedestrian streets are paving ribbons with no pavements; a junction
  where only pedestrian streets meet is paved too. Paths are 2 m paving
  ribbons laid 12 mm above the roads they cross and take no part in
  junction layouts. Streams are water ribbons 2 cm above the ground with
  no collider.
- Areas are flat polygons: water (2 cm, no collider), parks (2.5 cm,
  no collider, tinted grass), car parks (asphalt, 1 cm below the roads,
  collider), squares (paving, 12 mm above the roads, collider).
- Lamp posts stand in the middle of the pavement every `lamp_spacing`
  metres, alternating sides, with an arm and head over the road; they are
  colliders on the prop layer. `lamp_spacing = 0` turns them off.
- Buildings are extruded footprints: wall quads with outward normals and a
  texture repeat per `window_width` and `storey_height`, an ear-clipped
  roof. Facade variant is a stable hash of the source id. Both are
  colliders on the building layer.
- Ground tiles cover every chunk cell at y = 0 with grass.
- The spawn is the midpoint of the longest segment of the longest
  ordinary road whose middle lies in the central part of the extract,
  facing the end of that road nearer the centre of town.
- Output is deterministic for the same model and settings; `config_hash`
  in the manifest is the settings hash.

## Validation

`city-validate` checks every part has vertices and triangles, indices in
range, finite positions, unit normals and no degenerate triangles; that
vertices lie inside their chunk bounds; that the spawn is inside a chunk;
that there are road surfaces and colliders; and that source and licence
are recorded. It prints counts and exits 1 on any issue.
