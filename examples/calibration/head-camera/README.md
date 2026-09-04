# Synthetic head-camera replay

This fixture is generated data, not hardware evidence.  It contains 12
distinct `base_from_head_tilt` poses and exact observations satisfying

```text
base_from_head_tilt · head_tilt_from_camera · camera_from_target
  = base_from_target
```

The planted `head_tilt_from_camera` translation is
`[0.031, 0.048, 0.034] m`.  `expected.yaml` checks that the solver recovers it,
that the residual remains effectively zero, and that the pose set has full
rotational rank.  Reprojection values are plausible synthetic quality metadata
used to exercise the publication gate.
