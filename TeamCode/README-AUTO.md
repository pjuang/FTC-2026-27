# BIOBUZZ Autonomous (2026-27): student guide

Everything here lives in `TeamCode`. **BasicTeleop is unchanged.** All robot facts are in
**one file**, `config/RobotConfig.java`. Anything marked `TODO` or `TODO-measure` is a guess. Check it on the real robot.

## 1. What's here

```
org/firstinspires/ftc/teamcode/
  BasicTeleop.java              (unchanged template TeleOp)
  config/RobotConfig.java       <- EVERY robot assumption: names, directions, sizes, timings
  pedro/
    Constants.java              Pedro v3 setup: drivetrain, localizer choice, Foresight follower
    DriveEncoderImuLocalizer.java  default localizer: 4 drive encoders + IMU heading
    Tuning.java + procedures/   Pedro tuners (copied from the official v3 quickstart)
  subsystems/
    Drive.java                  wraps Pedro's Follower (drive through named Poses)
    Intake.java, Launcher.java  do nothing (and don't crash) if the hardware isn't configured
    OptionalHardware.java
  auto/
    FieldPoses.java             ONE table of RED poses (inches); BLUE = point-mirror
    BioBuzzAuto.java            timer-guarded step machine shared by all Autos
    RedAuto / BlueAuto          near start (audience wall, under our HIVE)
    RedFarAuto / BlueFarAuto    far start (against our alliance wall)
    LeaveOnlyAuto               fallback: no Pedro, no IMU; drive 14 in forward = LEAVE
src/simulated/...               ftc-sim robot (only built with -PftcSim)
src/testSimulated/...           headless sim runs of every Auto
robot-config/, scenarios/       ftc-sim robot + field + starting layouts
```

Organization follows RobToT: subsystems, paths built from named Poses, Red/Blue variants,
tuner OpModes. **Pedro version:** we use **Pedro Pathing v3** (`com.pedropathing:revhub:3.0.2`,
`tuning:1.0.1`). Older guides and RobToT use `FollowerConstants` / `FollowerBuilder`, which come
from Pedro 1.x/2.x. In v3 you write `new Follower(localizer, drivetrain, new Foresight(config))`
(see `Constants.create`). **Don't copy 2.x code into this project.**

## 2. Before the first run on the robot

1. **Driver Station config names.** `RobotConfig` expects the following:
   - Drive motors: `fLeft`, `fRight`, `bLeft`, `bRight`, with directions REVERSE, FORWARD, REVERSE, FORWARD (same as BasicTeleop).
   - IMU: `imu`.
   - Mechanisms: `intake` (a motor or CR servo), `launcher` (a motor with an encoder), and optionally `feeder`.

   If yours are different, change `RobotConfig`, not the other files.
2. **Hub orientation.** Set `HUB_LOGO` and `HUB_USB` (currently LOGO UP / USB FORWARD, which is a TODO).
3. **Localizer.** Set `LOCALIZER`:
   - `DRIVE_ENCODER_IMU`: the default. Works with no extra hardware.
   - `PINPOINT` or `OTOS`: if you have one. Fill in the PINPOINT_* or OTOS_* offsets first.
4. **Drive geometry.** Check `DRIVE_TICKS_PER_REV` (537.7 for a goBILDA 312 rpm motor), `WHEEL_DIAMETER_IN` (96 mm), and `ROBOT_LENGTH_IN` / `ROBOT_WIDTH_IN`.

## 3. Tuning order (Pedro tuners, robot on blocks first)

The tuners show up as OpModes. Open the tuner web page at `http://192.168.43.1:10158` on the
Control Hub's Wi-Fi.

1. **Mecanum Tuner.** Checks that each wheel spins the right way.
2. **Localizer.**
   - Drive encoders: push the robot 48 in forward, then 48 in sideways, while watching the pose in telemetry. Then set `FORWARD_MULTIPLIER` / `STRAFE_MULTIPLIER` to measured ÷ reported. `STRAFE_MULTIPLIER` is 0.8 because that matches the simulator. **Measure it.**
   - Pinpoint/OTOS: run 2a/2b instead.
3. **Foresight Tuner.** Measures top speeds, deceleration, and gains, and prints a code block. Paste it over the `foresightConfig` block in `Constants.java`. The current numbers came from the simulator. They are not for your robot.
4. **Tests.** Drive a few paths and check the end pose against a tape measure.

## 4. What the Autos do (manual TU06)

| Phase | What | Rule |
|---|---|---|
| Setup | Pick an alliance in the OpMode list. Place the robot at the pose shown in the INIT telemetry ("Place robot at"). Preloads go on the tiles touching the front of the robot. | §10.3.1.A.iv, §10.3.4 (preloads "in or on the ROBOT, or on the TILES contacting the ROBOT"); G304 (no fixed start area) |
| 1 | Drive under our HIVE (far variants first go around the frame) and shoot the preloads into the AUDIENCE CELL. In the sim this TIPS our HIVE. | Table 10-2 (TIP 20) |
| 2 | Only if at least `CYCLE_MIN_TIME_S` (14 s) is left: sweep our 4 GARDEN POLLEN off the audience wall, carry them around the HIVE frame, and shoot from the rear into the SCORING CELL (now the up one). | §10.3.1 (GARDEN staging); G417 (don't launch into a down CELL) |
| Park | With `PARK_RESERVE_S` (6 s) left, any step is cut short and the robot drives to our LOADING ZONE, ending off the wall. Everything stops 0.3 s before 30 s. | §10.1 (AUTO = 30 s); §10.5.4 + Table 10-2 (LEAVE 3, PARK 5); G403 |

`DO_GARDEN_CYCLE` (in `BioBuzzAuto`) ships as `true`, which is what the sim runs used. On the real
robot, set it to `false` until the first volley is reliable. The cycle assumes the HIVE has tipped.
In AUTO, POLLEN in a CELL scores 0 (Table 10-2). The cycle only pays off if it causes a TIP.

Coordinates in `FieldPoses` are in inches:
- The origin is the field centre.
- +X points toward the blue alliance wall and +Y toward the rear wall.
- Heading 0 = +X, and positive heading is counter-clockwise.

Only RED poses are written. BLUE is the point mirror `(-x, -y, h+180°)`. Every pose marked `TODO-measure` must be checked on a real field.

## 5. Simulator (ftc-sim 12.0.4, season biobuzz)

The sim is **only** added when you pass `-PftcSim`, so the normal robot build is unchanged.
Use `bash ./gradlew` because the template's `gradlew` isn't executable on Linux/macOS (or run `chmod +x gradlew`).

```
bash ./gradlew :TeamCode:assembleDebug                         # real robot build (no sim)
bash ./gradlew -PftcSim :TeamCode:dashboard                    # 3D dashboard: http://127.0.0.1:8765/
bash ./gradlew -PftcSim :TeamCode:testSimulatedDebugUnitTest --tests '*BioBuzzAutoSimTest*'
```

- **Dashboard.**
  1. Choose a scenario such as `biobuzz-auto-red-near`.
  2. Drag the robot to the pose that INIT telemetry shows.
  3. INIT and START the Auto.
- **Headless test.** Runs every Auto for 30 s in real time, about 35 s each. It writes the following to `TeamCode/build/sim-results/`:
  - `<Auto>.txt`: score, end pose, and LEAVE/PARK.
  - `<Auto>.csv`: true pose against Pedro's estimate.

The sim has some limits, so don't trust it blindly:
- Real time only. Pedro and `ElapsedTime` use the wall clock, so the sim can't run faster than real time.
- No Pinpoint or OTOS devices in the sim. Only the drive-encoder+IMU localizer can be tested there.
- The sim robot can't hold balls. That is why the shot sequence is: back off, spin up, then creep in with the intake on (`SHOT_BACKOFF_IN`). A robot with a hopper and feeder can set it to 0.
- The sim doesn't score LEAVE/PARK. The test computes them from the end pose using a LOADING ZONE rectangle that is TODO-measure.
- Drive motors in the sim reach full speed in 0.25 s (`BioBuzzSimRobot.DRIVE_SECONDS_TO_FULL_SPEED`). With "ideal" motors the wheels slip, and drive-encoder localization falls apart.
- Results vary a little from run to run because of thread timing.
