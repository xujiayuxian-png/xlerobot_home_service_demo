# Unit calibration

Calibration is local state, not a set of constants to copy from the reference
robot. The workflow produces immutable versions under
`.xlerobot/units/<unit>/versions/`, points `active` to one valid version, and
renders only a checksum-valid active bundle into `runtime/`.

The required sequence is:

1. Servo zero, direction, raw range, and joint range.
2. Base wheel radius and wheel separation.
3. Head D455 extrinsic from a fixed 4 x 4 AprilTag board.
4. Right-arm hand-eye transform from a tag attached to the gripper.
5. Grasp alignment measured on the final mechanical setup.

The printable PDF targets and matching SVG sources are in
[`assets/calibration_boards`](../../assets/calibration_boards/). Print the PDF
at 100%, mount it on a rigid flat surface, and measure the printed tag edge
before collection.

Copy `config/local.example.yaml` to the ignored `config/local.yaml` and set the
unit ID and stable device paths first. Live acquisition is routed through the
same public command and always requires explicit hardware consent:

```bash
./tools/calibrate capture servo --hardware
# Use the web page, finalize, stop it with Ctrl-C, then run the printed command.

./tools/calibrate capture head-camera --hardware
# Capture the board at the prescribed poses, stop, then run the printed command.

./tools/calibrate capture right-handeye --hardware
# Capture Tag 23 at the prescribed poses, stop, then run the printed command.
```

Each capture starts only one workflow and prints its local HMI URL plus the
exact follow-up command. Startup itself never moves the robot; clicking a
visual-calibration pose does. Support the arm before releasing servo torque.
All raw captures stay below
`.xlerobot/units/<unit>/capture/` and remain outside Git.

An existing capture is never mixed in silently. Use `--resume` only to append
to the same head-camera/hand-eye session. Use `--fresh` after a mechanical
change; it moves the preceding workflow directory into the local `archive/`
instead of deleting it.

Base geometry is a tape-measure workflow. Either fill in
`examples/calibration/base_measurements.yaml`, or use the web worksheet started
by `./tools/calibrate capture base --hardware`. The worksheet does not drive
the base; execute each straight/turn trial under direct supervision and enter
the commanded and measured values.

The strict public solver/validator then stores one draft component at a time.
The live page deliberately does not activate a bundle. The paths below use
the example unit ID `demo-01`; replace it if you changed `robot.unit_id`:

```bash
./tools/calibrate servo --input .xlerobot/units/demo-01/capture/calibration_work/servo/result.yaml
./tools/calibrate base --measurements /path/to/base_measurements.yaml
# Or import the result printed by the base capture page:
./tools/calibrate base --input .xlerobot/units/demo-01/capture/calibration_work/base_geometry/result.yaml
./tools/calibrate head-camera --samples .xlerobot/units/demo-01/capture/calibration_work/head_camera/samples.yaml
./tools/calibrate right-handeye --samples .xlerobot/units/demo-01/capture/calibration_work/right_handeye/samples.yaml
./tools/calibrate grasp-alignment --measurements /path/to/alignment.yaml
./tools/calibrate status
```

Each later capture must see the passing drafts that precede it. Render those
inputs into a staging directory without activating an incomplete bundle:

```bash
./tools/calibrate render --for head-camera      # requires servo
./tools/calibrate render --for right-handeye    # requires servo + head
./tools/calibrate render --for grasp-alignment  # requires all structural items
```

Base fitting uses commanded/actual tape-measure trials and can run in parallel
with the `servo → head-camera → right-handeye` chain. It becomes required when
rendering the grasp-alignment stage and when publishing the final bundle.

`tools/calibrate capture` performs this staged render automatically. The
rendered directory is `.xlerobot/units/<unit>/draft/runtime/<workflow>/`; its
manifest is marked `source: draft` and `final_demo_runtime: false`. The Demo
never reads this staging tree.

After every component passes validation, activate and render it:

```bash
./tools/calibrate activate
./tools/calibrate render
./tools/calibrate status
```

Plain `render` reads only the complete active bundle. It never promotes a
staged draft into the final sibling `runtime/` directory.

`tools/run demo --hardware` consumes only these rendered files and refuses to
continue if they are missing or invalid. It never reads component drafts
directly. Rollback is explicit:

```bash
./tools/calibrate rollback --version <version>
./tools/calibrate render
```

For a robot that already has a working calibration, `import-runtime` can
preserve its complete existing runtime in an immutable version. The input
directory must contain `geometry.yaml`, `servos.yaml`, `controllers.yaml`,
`transforms.yaml`, and `grasp_alignment.yaml`, with recorded provenance and
alignment marked `validation: existing_unit_runtime`. This checks executable
values and file integrity; it does **not** claim that old measurements passed
the new solver quality gates. Use it only for the same physical robot and
unchanged mounts. New builds should follow the measurement workflow above.

```bash
./tools/calibrate import-runtime --input PATH --version existing-unit-v1
```

Run the two solver fixtures without hardware:

```bash
./tools/calibrate replay head-camera
./tools/calibrate replay right-handeye
```

Passing replay proves the solver and file contract, not the calibration of your
robot. Do not commit `.xlerobot/`; it identifies one physical unit.
