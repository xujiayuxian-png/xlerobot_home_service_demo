# Grasping examples

`shared_rgbd_fixture.yaml` defines a small synthetic tabletop depth/mask scene
and expected geometry for the centroid/GPD contract tests. It is not a camera
recording or proof of a successful hardware grasp. The scene has no private
map or household image; the fixture is CC0-1.0.

For live use, follow [the grasp guide](../../docs/en/grasping.md)
([中文](../../docs/zh-CN/grasping.md)). The fixture is consumed by the perception
package's tests; it is not an input for `tools/run grasp`.
