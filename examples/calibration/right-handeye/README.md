# Anonymized real Tag23 replay

This fixture preserves the 36 accepted transform pairs and reprojection scores
from a physical AprilTag 36h11 ID 23 capture.  It intentionally excludes the
images, filesystem paths, wall-clock timestamps, requested/actual joint logs,
camera serial information, and host-specific configuration.

The replay solves `base_from_camera` and `gripper_from_target` together.
`expected.yaml` records the observed result with tolerances chosen to catch a
solver/frame regression while allowing small numeric differences between
supported OpenCV and SciPy builds.  The reference run has translation RMS about
6.26 mm and p95 about 12.25 mm.  Its 15.31 mm maximum remains reported rather
than being hidden; the publication rule deliberately gates on p95.
