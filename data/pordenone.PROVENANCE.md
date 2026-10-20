# data/pordenone.osm

- Source: OpenStreetMap, https://api.openstreetmap.org/api/0.6/map
- Bounding box: minlon 12.6540, minlat 45.9545, maxlon 12.6640, maxlat 45.9615
  (central Pordenone, about 780 m by 780 m)
- Downloaded: 2026-10-20
- Licence: Open Database License (ODbL) 1.0, © OpenStreetMap contributors,
  https://www.openstreetmap.org/copyright
- Not committed. Re-download with:

```
curl -A "gta-like-prototype/0.1" -o data/pordenone.osm \
  "https://api.openstreetmap.org/api/0.6/map?bbox=12.6540,45.9545,12.6640,45.9615"
```

# data/pordenone.elevation.txt

- Source: EU-DEM v1.1, 25 m (Copernicus Land Monitoring Service), read
  through the public Open Topo Data API, https://www.opentopodata.org
- Area: the bounding box above plus 100 m, sampled every 30 m
- Licence: free to use with attribution to the Copernicus programme
- Not committed. Re-fetch with:

```
python3 scripts/fetch_elevation.py 45.9545 12.6540 45.9615 12.6640 data/pordenone.elevation.txt
```
