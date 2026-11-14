# Rectangle map assets (0908 road width)

This folder contains a simple rounded-rectangle closed track using the 0908
median road width, cropped to the measured map1 free-space envelope:

- map size: 125 x 217 cells (6.25 m x 10.85 m)
- resolution: 0.05 m/cell
- origin: `[-4.73, -4.93, 0]`
- target road width: 1.960083 m (the median total width of the 0908 equal-width profile)
- target physical outer size: 6.15 m x 10.75 m
- centerline half extents: 2.095 m x 4.395 m
- corner radius: 1.35 m

The outer size is 0.10 m smaller than map1's measured 6.25 m x 10.85 m
free-space envelope. The rounded corners preserve the requested rectangular
layout while avoiding the infinite curvature of sharp 90-degree corners.

## Generation pipeline

1. `step_1_live_centerline_width_preview_gui.py --batch` measured the two
   boundaries from the occupancy map.
2. `step_2_refine_width_profile_gui.py` loading/saving functions validated and
   preserved the exact generated boundary points (no manual displacement was
   necessary for this synthetic map).
3. `step_3_refine_centerline_for_equal_width.py` generated
   `centerline_equal.csv`, the equal-width profiles, and `rectangle_mppi_track.csv`.
4. `step_4_generate_minimum_curvature_raceline.py` generated
   `rectangle_mppi_track_optimal.csv` for a 0.32-m-wide vehicle.

The measured final equal half-width has median 0.9751 m (the small difference
from 0.9800 m is the expected 0.05-m occupancy-grid discretization effect).
