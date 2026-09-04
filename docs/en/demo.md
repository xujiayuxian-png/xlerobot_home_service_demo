# Full fetch-and-deliver demo

The operator flow and web UI are visible in the
[HMI demonstration video](https://www.bilibili.com/video/BV1GSK66XEqf).

The reference sequence is:

```text
voice/web/CLI request -> localization -> navigate and dock at table
-> object grounding -> selected grasp route and verification
-> nearest-person search -> approach -> voice feedback -> handover (then ready)
```

## Preconditions

- The reference hardware modification and stable device aliases are present.
- A complete unit calibration is active and rendered.
- The Nav2 map and named-place file describe the current site and contain the
  `table` route expected by the task.
- LM Studio, classical port 8765, and ACT port 8766 are reachable from Robot.
- The configured person model exists when nearest-person delivery is used.
- Voice assets/models and audio devices are ready when `demo.voice` is true.

Run `tools/doctor robot`; resolve every `ERROR` before startup. Warnings for an
intentionally absent optional device should be reconciled with your config.

## Start

```bash
# GPU computer
./tools/run gpu

# Robot computer
./tools/run demo --hardware
```

The web console is at `http://<robot-host>:8080` by default. The default voice
and web task is ACT grasping of `羽毛球`. For a controlled backend comparison,
submit it from a second terminal:

```bash
./tools/run grasp act --hardware
```

Stop the foreground Robot process with Ctrl-C before changing calibration,
device wiring, map, or controller configuration. Stop `tools/run gpu` with
Ctrl-C to terminate both proposal services.
