# map1 reverse-direction map assets

These assets were regenerated through the map-utils pipeline rather than by
algebraically reversing the forward raceline CSV.

1. `map1_centerline.csv` supplied only the reverse-order geometric seed.
2. Step 1 re-measured left/right widths and boundaries from `map1_reverse.pgm`.
3. Step 2 loaded, validated, and saved the reverse-direction boundaries.
4. Step 3 rebuilt the equal-width centerline and
   `map1_reverse_mppi_track.csv`.
5. Step 4 optimized `map1_reverse_mppi_track_optimal.csv` directly in the
   reverse direction.

Step 4 uses a conservative 0.80 m optimization envelope. With the nominal
0.32 m F1TENTH vehicle width, the first candidate approached an internal map
obstacle too closely (diagnostic minimum normal clearance 0.025 m). The 0.80 m
envelope raises the final diagnostic clearance to 0.313 m left and 0.353 m
right while remaining feasible on this map.
