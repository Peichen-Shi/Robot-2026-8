# Upstream and derivative changes

Repository: https://github.com/Peichen-Shi/Robot-2026-8/tree/main/lab4

Pinned commit: 8f8f7af6668fbb2b51136e9c6570a54eb09300c7

The original author supplied the geometric controller, minimum-snap planner,
TESSE bridge, gate timer, analysis and racing acceptance scripts. Original
notices and explanatory comments are retained. A source patch and upstream
manifest are included.

This revision adds bounded gate-local aperture offsets and causal reference
prediction to the measurement epoch. The user authorized choosing points
within the original gate apertures. Physical gates, their order, yaw, aperture
checks, collisions, clock rate and simulator constants remain unchanged.
Independent spline audits reconstruct selected points; actual crossings are
still judged against the original physical gate frames.

The final gains remain kx=16, kv=8, kr=24 and komega=4. The thrust command
limit rises from 50 to 75, x/y moment limits from 8 to 12 and z moment from
5 to 7. Mass, inertia, gravity, rotor coefficients and the Unity binary do
not change. Original watchdogs and race acceptance thresholds remain enabled.
An eight-second observation follows reference end; first-to-last gate timing
excludes this observation.

Unity's rendered 3D window is the default. Headless and RViz modes remain
optional diagnostics; every recorded race identifies its startup mode.
