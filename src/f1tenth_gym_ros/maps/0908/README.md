# 0908 map assets

These assets are copied from `data/0903` for the September 8 recording session.
The `0903` filename prefixes and the map YAML's image reference are renamed to
`0908`.

All eight bags under
`/home/a/Desktop/shihoon/2027 ICRA/bag_files/0908/`
contain the same `/map` occupancy grid as `data/0903/0903.yaml`: 177 by 241 cells,
0.05 m resolution, and origin `[-6.03, -5.53, 0]`. The centerline, width profiles,
and MPPI track CSVs are copies of the existing 0903 assets; they were not
regenerated from the new driving trajectories.

Step 1 saves the new session's extracted NPZ/JSON files in
`model_tuning/data/0908`. Step 7 writes rebuilt KF data to
`model_tuning/data/0908_collision_refined_current_kf_gt`. Steps 2, 3, and 4 save
their regression results under `model_tuning/results/0908`.

For the first identification pass, run Step 1 -> Step 2 -> Step 3 -> Step 4 ->
Step 7. Steps 2, 3, and 4 read the saved Step-1 files in
`model_tuning/data/0908`; Step 7 then rebuilds KF states with the parameters
accepted into `config/params.yaml`. To repeat classic fitting against refreshed
KF states, explicitly select the Step-7 output directory in Step 3.

Step 3 selects `DE_DEVICE = "cuda"` for the existing `de_robust_ls` backend.
The vehicle rollouts and loss reductions use float64 PyTorch CUDA; SciPy still
runs the DE and least-squares optimizers. Window selection, fitting bounds,
the objective and deployment checks are unchanged. `DE_DEVICE = "cpu"` selects
the reference NumPy path. Restart Step 3 to apply a device change; an already
running process keeps its original backend. DE now prints progress per generation.

To verify numerical parity and measure objective evaluation time without
fitting the real data or updating YAML, run:

```bash
/usr/bin/python3 model_tuning/real_car_v2/check_classic_cuda.py --device cuda --benchmark
```
