# data/pordenone.osm

- Source: OpenStreetMap, https://api.openstreetmap.org/api/0.6/map, fetched in
  tiles and merged by `scripts/fetch_osm.py` (the API refuses large boxes)
- Bounding box: minlon 12.648, minlat 45.949, maxlon 12.712, maxlat 45.992
  (central Pordenone to central Cordenons, about 5.0 km by 4.8 km)
- Downloaded: 2026-10-21 (160 688 nodes, 18 355 ways, 266 relations)
- Licence: Open Database License (ODbL) 1.0, © OpenStreetMap contributors,
  https://www.openstreetmap.org/copyright
- Not committed. Re-download with:

```
python3 scripts/fetch_osm.py 12.648 45.949 12.712 45.992 data/pordenone.osm
```

The earlier extract of central Pordenone alone (780 m by 780 m, downloaded
2026-10-20) is kept as `data/pordenone_centre.osm`.

# data/pordenone.elevation.txt

- Source: EU-DEM v1.1, 25 m (Copernicus Land Monitoring Service), read
  through the public Open Topo Data API, https://www.opentopodata.org
- Area: the bounding box above plus 100 m, sampled every 40 m
- Licence: free to use with attribution to the Copernicus programme
- Not committed. Re-fetch with:

```
python3 scripts/fetch_elevation.py 45.949 12.648 45.992 12.712 data/pordenone.elevation.txt 40
```
