# data/barletta.osm

- Source: OpenStreetMap, https://api.openstreetmap.org/api/0.6/map, fetched in
  tiles and merged by `scripts/fetch_osm.py`
- Bounding box: minlon 16.262, minlat 41.306, maxlon 16.306, maxlat 41.336
  (Barletta old town, the castle and the sea front, about 3.7 km by 3.3 km)
- Downloaded: 2026-10-21 (58 787 nodes, 7 310 ways, 302 relations)
- Licence: Open Database License (ODbL) 1.0, © OpenStreetMap contributors,
  https://www.openstreetmap.org/copyright
- Not committed. Re-download with:

```
python3 scripts/fetch_osm.py 16.262 41.306 16.306 41.336 data/barletta.osm
```

The ground is flat: no elevation model was fetched for this city.
