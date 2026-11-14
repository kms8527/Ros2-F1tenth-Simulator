# 0908 reverse-direction map assets

These files were regenerated through the map-utils pipeline, not produced by
algebraically reversing the final forward CSVs.

1. The original geometric centerline was used only as a reverse-order seed.
2. Step 1 ray-marched the occupancy map again and measured reverse-direction
   left/right widths and explicit boundary coordinates.
3. Step 2 loaded, validated, and saved those reverse-direction boundaries
   without manual displacement.
4. Step 3 rebuilt the equal-width reverse centerline and
   `0908_reverse_mppi_track.csv`.
5. Step 4 ran the minimum-curvature optimizer directly on that reverse track
   and created `0908_reverse_mppi_track_optimal.csv`.

The occupancy geometry is identical to `data/0908`; only the ROS map filename
and direction-dependent track assets differ.
