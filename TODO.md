# TODO

## The distributed optimiser blocks LiDAR odometry while it re-checks loop closures

**Status:** open, not fixed on purpose in the thesis runs (pinned at `4dd5b47`): the stall is upstream DCL-SLAM
behaviour, so the experiments measure it as it is. This note is for improving the code later, here and upstream.

**What happens.** At the start of every distributed-optimisation round, `run()` calls
`initializePoseGraphOptimization()`, which reloads the pose graph (`updateOptimizer()`) and re-runs pairwise
consistency maximisation over every inter-robot loop closure the robot holds (`outliersFiltering()`). `run()` holds
`state_mutex` for the whole step (`src/distributedMapping.cpp`, `run()` around line 1345), and LiDAR odometry needs
the same lock for every processed scan (`publishTransformation()`, and `performDistributedMapping()` on keyframes).
LIO-SAM's scan subscription keeps one waiting scan, so every scan that arrives during the step except the newest is
dropped.

The step grows with the number of closures. Six simulated robots (Gazebo, VLP-16-like LiDAR, about 15 min missions,
170-230 accepted inter-robot closures per robot by the end) measured from the optimiser's state messages:

- median 0.10-0.12 s in the first 300 s of the mission;
- 0.79 s after 500 s for the robots with the most closures, up to 2.6 s;
- those robots lost 3-15 % of their LIO poses, all in the last third of the mission, in every repetition and at
  every machine load tried.

Upstream (`378d187`) is no better: the round timer runs on the default callback queue, i.e. on the `ros::spin()`
thread together with `laserCloudInfoHandler`, so the whole optimiser blocks LIO. This fork moved the round to its
own queue and thread (`dgs_queue`, `distributedMappingBasis.cpp` around line 254), but the shared `state_mutex`
(added because loop-closure inserts reallocated the graph while the optimiser iterated it, about 1 crash in 6 runs)
keeps this one step blocking. Removing the lock would bring the crashes back.

**Why it matters beyond accuracy.** Fabricated or wrong loop closures enter the same check, so anything that adds
closures (including an attacker) also lengthens the stalls and degrades odometry as a side effect.

**Possible fixes (measure first).**

1. Add timers around `updateOptimizer()` and `outliersFiltering()` to see which part dominates.
2. Run the check outside the lock: under `state_mutex`, copy the graph, the local map and the neighbours' estimates;
   release it; run PCM on the copy; take the lock again only to apply the removals. LIO only appends factors, so the
   indices of the copied factors stay valid.
3. Make PCM cheaper (in `distributed_mapper`): it checks all O(N^2) pairs of a neighbour's closures every round,
   `composeOnTrajectory` copies the trajectory map by value for each check, and the consistency matrix is written to
   `consistency_matrix_<robot>.clq.mtx` and read back before the max-clique search. Checking only new closures
   against the current clique, passing the trajectory by reference and keeping the matrix in memory would each help.
4. Not a fix: a longer LIO scan queue keeps the scans but delivers poses up to 2.6 s late.

Evidence and scripts: `slam-fdi-attacks`, `docs/t0_baseline_2026-11-16/investigations/D4/REPORT.md`.
