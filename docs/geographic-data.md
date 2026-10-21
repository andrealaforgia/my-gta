# Geographic data

## Source and licence

OpenStreetMap data, Open Database License 1.0. Attribution "© OpenStreetMap
contributors" is required wherever the generated city is shown or
distributed, and ODbL share-alike may apply to a distributed derived
database. The package manifest carries the licence line and the game logs
it at load. Review the obligations before any public release.

- https://www.openstreetmap.org/copyright
- https://opendatacommons.org/licenses/odbl/

Raw extracts live in `data/` and are not committed; each has a
`<name>.PROVENANCE.md` next to it with the URL, bounding box, date and
licence. The committed fixture `tests/fixtures/district.osm` is
hand-written, not OSM data.

## Getting an extract

The OSM API serves small bounding boxes as XML:

```
curl -A "gta-like-prototype/0.1" -o data/<name>.osm \
  "https://api.openstreetmap.org/api/0.6/map?bbox=<minlon>,<minlat>,<maxlon>,<maxlat>"
```

Keep it to a few blocks (under about a kilometre square). PBF is not
supported (ADR-004); convert with `osmium cat x.osm.pbf -o x.osm` if you
have a PBF.

## What the importer reads

- Ways tagged `highway` with a drivable class: motorway, trunk, primary,
  secondary, tertiary, residential, living_street, unclassified, road
  (and their `_link` forms). Service roads are kept when they are alleys
  or plain service streets; driveways, car park aisles, drive-throughs,
  emergency accesses and anything tagged `access=private|no` are skipped
  (`--no-service-roads` drops them all): they clutter blocks without
  adding anywhere to drive.
- Ways with `tunnel` other than `no` (underpasses, `building_passage`)
  are not drawn at all. On the surface they ran across gardens and
  rooftops; in Pordenone 46 ways were skipped for this.
- `highway=pedestrian` ways become the `pedestrian` class: a paved street
  the whole width across, no separate pavements. With `area=yes` they are
  squares (areas, below).
- Footways, cycleways, paths, steps and bridleways become the `path`
  class: a narrow paved strip laid over whatever it crosses, with no
  junction geometry. Footways tagged `footway=sidewalk` or
  `footway=crossing` are skipped because the generated pavements already
  stand for them.
- Open ways tagged `waterway` stream, river, canal, ditch or drain become
  the `waterway` class: a water ribbon with no collider (streams 2.5 m,
  canals 5 m, rivers 12 m unless `width` says otherwise).
- Width from `width`, else `lanes` x 3.25 m, else a configurable default
  per class, and the result is marked inferred.
- `oneway`, `layer`, `bridge`, `tunnel`, `surface`, `name` are kept as
  metadata.
- Ways tagged `building` that are closed rings with area. Height from
  `height`, else `building:levels` x 3.2 m, else the typical height of
  its kind (house and terrace two storeys, hut and garage 3 m, apartments
  five, church 16 m), else the median of tagged buildings within 80 m
  varied by a storey up or down by the way id, else two to four storeys.
  Rings are made anticlockwise.
- Single nodes tagged `natural=tree` become street trees (a trunk with a
  collider and a crown). Benches (`amenity=bench`), bus stops, bollards and
  traffic signals are kept as street furniture too; other node tags are not
  read. Pordenone has 37 trees, 16 benches, 6 bus stops, 5 bollards and 7
  signals. Signals show red, amber and green on a 48 s cycle and cars stop at
  them; a mapped signal node faces both ways along its road. Busy junctions
  without a mapped signal get lights of their own (see architecture.md).
- Ways tagged `barrier=wall`, `retaining_wall`, `city_wall`, `fence` or
  `hedge` become low solid slabs (1.9 m of stone, 1.25 m of timber, 1.3 m of
  hedge) with a collider, written to `barriers.txt` in the intermediate
  model. Pieces that would lie on a carriageway are left out so a mapped
  wall never blocks a street; gates and other barrier kinds are ignored.
  Pordenone has 19 such ways.
- Interior corners of roads are replaced by arcs (quadratic, up to 8 m
  back from the corner) and two segments meeting end to end at a node
  nothing else uses, with the same class, width and direction rules, are
  joined into one road first, so bends are not junctions.
- Areas: closed ways and multipolygon relations tagged `natural=water`,
  `waterway=riverbank`, `landuse=reservoir|basin` (water); `leisure=park`
  and friends, `landuse=grass|forest|meadow|...`, `natural=wood|scrub`
  (park); `amenity=parking` (parking); `highway=pedestrian` with
  `area=yes` or `place=square` (square).
- Multipolygon relations tagged `building` or with an area tag have their
  `outer` member ways chained end to end into closed rings; each ring is
  a building or area with the relation's tags. `inner` rings of a
  building become courtyards of the outer ring that contains them, cut
  out of the roof with their own inward-facing walls. Rings that do not
  close are counted and skipped with a warning.

## Cleaning and the road graph

Consecutive repeated nodes and coincident points are removed; ways with
fewer than two distinct points or shorter than 0.5 m are dropped as
degenerate. A way whose end stops within two metres of
another way without sharing a node is snapped onto it: to that way's
vertex if one is in reach, otherwise onto the segment, which is split
there with a fresh node. Mappers leave such gaps and they would otherwise
become roads crossing without a junction. Ways are then split at every
node shared with another accepted way, so each road segment runs
junction to junction. A junction records
its degree, the number of segment ends meeting there. Ways on different
layers only connect where they share a node, which is how OSM models it.

## Coordinates

Equirectangular projection about the centre of the extract's bounds:
x = east metres, z = south metres (north is -z), y up. Within a few
kilometres this is accurate to well under a metre.

## Elevation

OpenStreetMap carries no ground height, so the hills come from a separate
lattice: `scripts/fetch_elevation.py MIN_LAT MIN_LON MAX_LAT MAX_LON
OUT [SPACING_M]` samples the EU-DEM 25 m model (Copernicus, free with
attribution) through the public Open Topo Data API every 30 m, with 100 m
of margin, and writes `grid` and `row` lines. `city-import --elevation
FILE` projects it onto the model's local grid, smooths it with one 3x3
box pass (the model is a surface model, so rooftops leak in), makes it
relative to its lowest sample and writes `terrain.txt` beside the model.
`make city` passes `data/<name>.elevation.txt` when it exists. Pordenone
falls 21 m from the north west to the river in the south east. Footways tagged `bridge=yes` with `layer=1` are raised: the deck climbs
along a smooth ramp from each end to 0.25 m plus 4 cm per metre of its
length (at most 4.5 m), with a parapet and a slab underneath. Bridges for cars
stay at ground level.
