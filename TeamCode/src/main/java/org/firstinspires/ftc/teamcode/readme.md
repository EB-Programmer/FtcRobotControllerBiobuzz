# TeamCode

Robot code for BIOBUZZ. Driving and autonomous paths use [Pedro Pathing](https://pedropathing.com/)
2.0.2 with a goBILDA Pinpoint for odometry.

## OpModes

| OpMode | File | What it does |
|---|---|---|
| EB Auton | `BiobuzzAuton.java` | Runs a route chosen from a menu during INIT, with live telemetry and an end-of-run summary |
| EB Teleop | `BiobuzzTeleop.java` | Driver control, plus the shooter, intake, and Limelight auto-aim at the HIVE (controls listed at the top of the file) |
| Tuning | `pedro/Tuning.java` | Pedro Pathing's tuning OpModes |
| EB Auton: Rotate 45 CW | `examples/ExampleRotateAuton.java` | Turn-in-place test |

## Packages

- `pedro/` -- Pedro Pathing setup. `Constants.java` holds the drivetrain motor names and directions,
  Pinpoint pod offsets, path constraints, and follower constants (mass and PIDFs, still untuned).
- `pedro/paths/` -- one class per route's paths, pasted from the online path visualizer. Only
  pasted code goes here, so a route can be re-pasted after changing it in the visualizer.
- `routes/` -- one class per route (`Route`), pairing its pasted paths with what the robot does at
  the start and after each path, using the helpers in `RouteActions`.
- `subsystems/` -- robot mechanisms: `Shooter`, `Intake`, and `HiveAim` (Limelight auto-aim) with its helpers
  `TagCorners`, `TagCluster`, `HiveMemory`, `LimelightMount`, and `TagDebugTelemetry`.
- `util/` -- general helpers: the INIT settings menu (`InitMenu`), the INFO/DEBUG telemetry switch
  (`LogLevel`; X + d-pad left/right on either gamepad, in both OpModes), what EB Auton hands on
  to EB Teleop (`MatchSettings`: the alliance and where the robot ended up), follower telemetry,
  pose history, loop timing, and `Utils`.
- `com/pedropathing/` (next to `org/`) -- a small compatibility layer that lets the path visualizer's
  output, written for Pedro Pathing 3.0, compile against 2.0.2. **If this project upgrades to Pedro
  Pathing 3.0, delete it**: its classes would clash with the real 3.0 ones.

## Hardware configuration names

| Name | Device | Required? |
|---|---|---|
| `leftFrontDrive`, `rightFrontDrive`, `leftRearDrive`, `rightRearDrive` | drive motors | yes |
| `pinpoint` | goBILDA Pinpoint | yes |
| `limelight` | Limelight 3A | no -- teleop auto-aim is disabled without it |
| `shooter` | shooter motor | no -- teleop shooter controls are disabled without it |
| `intake` | intake motor | no -- intake controls and route intake actions do nothing without it |

## Limelight pipeline settings

Auto-aim uses pipeline slot 4, set up in the Limelight's web interface (connect the Limelight to a
laptop by USB and browse to `http://limelight.local:5801`):

- **Standard tab:** Family *AprilTag Classic 36h11*, Marker Size **82.55 mm** (3.25 in).
- **Advanced tab:** **Full 3D** on. Without it, auto-aim still works on tags in view but can't
  remember where the HIVE is.
- **Output tab:** corner output on. Without it, upside-down tags can't be spotted, and the Aim
  telemetry line warns "NO CORNERS".

## Adding an autonomous route

See `pedro/paths/package-info.java`. In short: paste the visualizer's code into a new class in
`pedro/paths/`, make a matching `Route` in `routes/` (see `UpCellRoute`), and add it to the
`routes` list at the top of `BiobuzzAuton`.

## Constants worth knowing about

- `BiobuzzAuton.MAX_POWER` -- top drive power while following auton paths.
- `HiveAim` -- auto-aim tuning: `KP`, `MIN_TURN`, `MAX_TURN`, `TOLERANCE_DEG`, and the Limelight
  pipeline slot.
- `LimelightMount` -- where the camera is mounted (forward/left offset and upward tilt).
- `TagCluster.OFFSETS_IN` -- AprilTag spacing within a cluster, in ID order as seen with the CELL up.
