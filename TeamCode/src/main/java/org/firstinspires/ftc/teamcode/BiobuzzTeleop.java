package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HiveAim;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGates;
import org.firstinspires.ftc.teamcode.util.FollowerTelemetry;
import org.firstinspires.ftc.teamcode.util.LogLevel;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.util.List;

/*
 * Controls for Gamepad 1:
 *   Left Stick:     Move forward & backward, strafe left & right. In driver's view drive,
 *                   moves relative to the driver standing at the red side of the field instead:
 *                   up drives away from the driver, left drives to the driver's left
 *   Left Trigger:   Turn counterclockwise
 *   Right Trigger:  Turn clockwise
 *   X + Left Trigger: Toggle driver's view drive (hold X, then pull the trigger). The triggers
 *                   don't turn while X is held. Needs the pose from EB Auton to know which way
 *                   is away from the driver; otherwise "away" is the way the robot faced at START
 *   A:              Fast Drive Mode
 *   B:              Slow Drive Mode
 *   Y:              Auto-aim at the HIVE (hold). Overrides turning with the triggers.
 *
 * Controls for Gamepad 2:
 *   Left Bumper:    Reverse intake (press to turn on or off)
 *   Right Bumper:   Intake (press to turn on or off)
 *   Right Trigger:  Shoot (hold)
 *   Dpad Up/Down:   Increase / decrease shooter speed, 100 RPM a press
 *   Dpad Right/Left: Increase / decrease gate 1's position, for finding the values to put in
 *                   ShooterGates. The first press moves it to the middle (0.5) if the position
 *                   isn't set yet. Ignored while X is held (X + d-pad sets the telemetry level)
 *   A:
 *   B:
 *   Y:
 *
 * Either gamepad:
 *   X + B:          Toggle alliance (Red / Blue): hold X, then press B. Can be done during
 *                   INIT. Starts on the alliance chosen in EB Auton, or Red if EB Auton
 *                   hasn't run
 *   X + Dpad Left:  Telemetry at INFO (the default)
 *   X + Dpad Right: Telemetry at DEBUG (adds per-tag, motor power, and joystick lines)
 *
 * After EB Auton, odometry picks up from where the auton left the robot, so poses are in field
 * coordinates; otherwise it starts at x 0, y 0, heading 0 wherever the robot is.
 *
 * The shooter, the intake, the Limelight, and the indicator lights are optional hardware: without
 * a "shooter" or "intake" motor, a "limelight", or "ledIndicator0" and "ledIndicator1" configured,
 * everything else still works and that hardware's controls do nothing. The lights are green while
 * tags aiming can use are in view, otherwise off.
 */

@TeleOp(name = "EB Teleop", group = "EBBiobuzz")
public class BiobuzzTeleop extends LinearOpMode {
    private static final double DRIVE_HIGH_POWER = 1.0;
    private static final double DRIVE_LOW_POWER = 0.4;

    // Static so a value tuned with the d-pad carries over to the next run, until the robot
    // app restarts
    private static double shooterRpm = 3000;

    private RobotHardware hardware;
    private Follower follower;
    private Mecanum drivetrain;
    private Shooter shooter;
    private Intake intake;
    private HiveAim hiveAim;
    private final LogLevel logLevel = new LogLevel();
    private boolean fastDriveMode = true;
    // Whether the left stick moves the robot relative to the driver rather than to the robot's
    // front
    private boolean driverViewDrive = false;
    private boolean leftTriggerWasDown = false;
    private boolean redAlliance = true;
    // Where redAlliance and the starting pose came from, for telemetry. While the alliance is
    // still the default, its note is a reminder of how to change it instead.
    private String allianceSource = "X + B to change";
    private String poseSource = "started at 0, 0";
    // Whether odometry is in field coordinates, which it is only when picked up from EB Auton
    private boolean poseOnField = false;

    @Override
    public void runOpMode() {
        initHardware();

        // Wait for the game to start (driver presses START). Alliance can be chosen meanwhile.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);  // for Utils.column()
        while (opModeInInit()) {
            toggleAlliance();
            logLevel.update(gamepad1, gamepad2);
            follower.update();  // keeps odometry current, for remembering where the HIVE is
            hardware.updateHiveAim(redAlliance, false);
            hardware.addInitTelemetry(telemetry);
            addPoseTelemetry();
            addAimTelemetry();
            telemetry.addData(Utils.column("Log"), logLevel.describe());
            telemetry.update();
        }
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
        if (isStopRequested()) {
            hardware.stop();
            return;
        }

        follower.startTeleopDrive();
        // setTeleOpDrive() reads the robot's heading from what the follower's last update() saw
        // once teleop drive is on, so one update() is needed before drive() calls it the first
        // time -- otherwise that first call hits a null pose and crashes
        follower.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            toggleAlliance();
            logLevel.update(gamepad1, gamepad2);
            hardware.updateHiveAim(redAlliance, gamepad1.y);
            drive();
            shoot();
            tuneGate();
            runIntake();
            updateTelemetry();
        }

        hardware.stop();
    }

    private void initHardware() {
        poseOnField = MatchSettings.autonEndPose != null;
        hardware = new RobotHardware(hardwareMap, telemetry, poseOnField);
        follower = hardware.follower;
        shooter = hardware.shooter;
        intake = hardware.intake;
        hiveAim = hardware.hiveAim;
        drivetrain = (Mecanum) follower.getDrivetrain();
        takeMatchSettings();
    }

    /** Picks up the alliance and robot pose that EB Auton left in MatchSettings, if any. */
    private void takeMatchSettings() {
        if (MatchSettings.redAlliance != null) {
            redAlliance = MatchSettings.redAlliance;
            allianceSource = "from auton";
        }
        if (MatchSettings.autonEndPose != null) {
            follower.setStartingPose(MatchSettings.autonEndPose);
            MatchSettings.autonEndPose = null;
            poseSource = "from auton";
        }
    }

    /** Hold X and press B, on either gamepad, to switch alliance. */
    private void toggleAlliance() {
        for (Gamepad gamepad : new Gamepad[]{gamepad1, gamepad2}) {
            // Read every pass, so a B press made without X held is used up rather than counted
            // once X is pressed
            boolean bPressed = gamepad.bWasPressed();
            if (bPressed && gamepad.x) {
                redAlliance = !redAlliance;
                allianceSource = "set with X + B";
            }
        }
    }

    private void drive() {
        // Check if FastMode is being toggled on or off
        if (gamepad1.a) {
            fastDriveMode = true;
        } else if (gamepad1.b && !gamepad1.x) {
            fastDriveMode = false;
        }

        // Hold X and pull the left trigger to switch driver's view drive on or off. The trigger
        // is checked every pass so each pull counts once, as it starts.
        boolean leftTriggerDown = gamepad1.left_trigger > 0.5;
        if (gamepad1.x && leftTriggerDown && !leftTriggerWasDown) {
            driverViewDrive = !driverViewDrive;
        }
        leftTriggerWasDown = leftTriggerDown;

        // The left stick moves the robot fwd/back and strafes left/right
        // The triggers turn the robot counterclockwise and clockwise, except while X is held, so
        // X + left trigger doesn't spin the robot
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.x ? 0 : gamepad1.left_trigger - gamepad1.right_trigger;

        // Auto-aim replaces the driver's turn input while gamepad1 Y is held, when it has
        // something to aim at (see HiveAim.isControllingTurn)
        if (hiveAim.isControllingTurn()) {
            turn = hiveAim.getTurn();
        }

        double powerLimit = fastDriveMode ? DRIVE_HIGH_POWER : DRIVE_LOW_POWER;

        // If turning during Low Power Mode: decrease max speed even more for fine-tune aiming
        if (!fastDriveMode && Math.abs(drive) < 0.01 && Math.abs(strafe) < 0.01) {
            powerLimit /= 2;
        }

        // setMaxPower scales all wheel powers down proportionally (rather than clipping) when
        // the requested drive exceeds it, so slow mode stays proportional to the joystick input.
        follower.setMaxPower(powerLimit);
        // Robot centric: forward is the robot's front. Otherwise Pedro Pathing's field centric
        // drive, where forward is the field's +x and left its +y -- in the red alliance's field
        // coordinates, which every OpMode uses, that's away from the driver and to their left.
        follower.setTeleOpDrive(drive, strafe, turn, !driverViewDrive);
        follower.update();
    }

    /** The right trigger runs the shooter while held; the d-pad tunes how fast it runs. */
    private void shoot() {
        boolean speedUpPressed = gamepad2.dpadUpWasPressed();
        boolean speedDownPressed = gamepad2.dpadDownWasPressed();
        if (!shooter.isPresent()) {
            return;
        }
        shooterRpm = Utils.tuneConstant(
                shooterRpm, speedUpPressed, speedDownPressed, 100, Shooter.MAX_RPM);
        shooter.setRpm(gamepad2.right_trigger > 0.25 ? shooterRpm : 0);
    }

    /**
     * The d-pad's right and left nudge gate 1's position, for the state it's in. Skipped while X
     * is held, since X + d-pad left/right sets the telemetry level.
     */
    private void tuneGate() {
        boolean upPressed = gamepad2.dpadRightWasPressed();
        boolean downPressed = gamepad2.dpadLeftWasPressed();
        ShooterGates.Gate gate = hardware.gates.gate1;
        if (!gate.isPresent() || gamepad2.x || !(upPressed || downPressed)) {
            return;
        }
        double position = gate.getPosition();
        gate.setPosition(Double.isNaN(position)
                ? 0.5
                : Utils.tuneConstant(position, upPressed, downPressed, 0.01, 1.0));
    }

    /**
     * Each bumper turns its direction on with one press and off with the next; pressing one while
     * the other direction is running switches straight over.
     */
    private void runIntake() {
        boolean forwardPressed = gamepad2.rightBumperWasPressed();
        boolean reversePressed = gamepad2.leftBumperWasPressed();
        if (!intake.isPresent()) {
            return;
        }
        Intake.State state = intake.getState();
        if (forwardPressed) {
            intake.set(state == Intake.State.FORWARD ? Intake.State.OFF : Intake.State.FORWARD);
        } else if (reversePressed) {
            intake.set(state == Intake.State.REVERSE ? Intake.State.OFF : Intake.State.REVERSE);
        }
    }

    /** A caption for a line shown on both screens: lined up in a column on the init screen. */
    private String caption(String name) {
        return opModeInInit() ? Utils.column(name) : name;
    }

    private void addPoseTelemetry() {
        telemetry.addData(caption("Pose"), "%s (%s)",
                FollowerTelemetry.describePose(follower.getPose()), poseSource);
    }

    /** Which alliance is aimed at and what the camera sees, shown during init and driving. */
    private void addAimTelemetry() {
        telemetry.addData(caption("Aim"), "%s (%s), %s",
                redAlliance ? "RED" : "BLUE", allianceSource, hiveAim.describeStatus());
        telemetry.addData(caption("HIVE"), hiveAim.describeHive());
        if (logLevel.isDebug()) {
            hiveAim.addTagTelemetry(telemetry);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Fast Drive Mode", fastDriveMode);
        if (!intake.isPresent()) {
            telemetry.addData("Intake", "not found -- intake controls disabled");
        }
        if (shooter.isPresent()) {
            telemetry.addData("Shooter Target RPM", "%.0f", shooterRpm);
            telemetry.addData("Shooter RPM", "%.0f", shooter.getRpm());
        } else {
            telemetry.addData("Shooter", "not found -- shooter controls disabled");
        }
        ShooterGates.Gate gate1 = hardware.gates.gate1;
        if (gate1.isPresent()) {
            double position = gate1.getPosition();
            telemetry.addData("Gate 1 Position", Double.isNaN(position)
                    ? "not set (d-pad right/left to move)"
                    : String.format("%.2f (%s)", position, gate1.isOpen() ? "open" : "closed"));
        } else {
            telemetry.addData("Gate 1", "not found -- gate tuning disabled");
        }
        addPoseTelemetry();
        addAimTelemetry();

        if (logLevel.isDebug()) {
            List<DcMotorEx> motors = drivetrain.getMotors();
            telemetry.addData("Front Left Power", motors.get(0).getPower());
            telemetry.addData("Front Right Power", motors.get(2).getPower());
            telemetry.addData("Rear Left Power", motors.get(1).getPower());
            telemetry.addData("Rear Right Power", motors.get(3).getPower());

            telemetry.addData("gamepad1 LeftStick Y (-drive)", gamepad1.left_stick_y);
            telemetry.addData("gamepad1 LeftStick X (strafe)", gamepad1.left_stick_x);
            telemetry.addData("gamepad1 RightTrigger (turn)", gamepad1.right_trigger);
            telemetry.addData("gamepad1 LeftTrigger (-turn)", gamepad1.left_trigger);
        }

        telemetry.addLine();
        telemetry.addData("Stick Drive", !driverViewDrive
                ? "robot's front (X + left trigger to switch)"
                : poseOnField
                ? "driver's view (X + left trigger to switch)"
                : "driver's view -- no pose from auton, so 'away' is the way the robot faced at"
                        + " START");
        logLevel.addTelemetry(telemetry);
        telemetry.update();
    }
}
