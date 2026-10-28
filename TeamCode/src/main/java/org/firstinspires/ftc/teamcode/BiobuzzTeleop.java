package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.HiveAim;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.FollowerTelemetry;
import org.firstinspires.ftc.teamcode.util.LogLevel;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.util.List;

/*
 * Controls for Gamepad 1:
 *   Left Stick:     Move forward & backward, strafe left & right
 *   Left Trigger:   Turn counterclockwise
 *   Right Trigger:  Turn clockwise
 *   A:              Fast Drive Mode
 *   B:              Slow Drive Mode
 *   Y:              Auto-aim at the HIVE (hold). Overrides turning with the triggers. Aims
 *                   with the HIVE AprilTags when they're visible, otherwise turns toward where
 *                   the HIVE was last seen. If it gets there and still can't see any tags, the
 *                   triggers can turn again until Y is released or tags come into view
 *
 * Controls for Gamepad 2:
 *   Left Bumper:    Reverse intake (press to turn on or off)
 *   Right Bumper:   Intake (press to turn on or off)
 *   Right Trigger:  Shoot (hold)
 *   Dpad Up/Down:   Increase / decrease shooter power
 *   A:
 *   B:              Toggle alliance (Red / Blue). Can be pressed during INIT. Starts on the
 *                   alliance chosen in EB Auton, or Red if EB Auton hasn't run. Ignored
 *                   while Start is held, so Start + B (making a controller gamepad 2) doesn't
 *                   change it
 *   Y:
 *
 * Either gamepad:
 *   X + Dpad Left:  Telemetry at INFO (the default)
 *   X + Dpad Right: Telemetry at DEBUG (adds per-tag, motor power, and joystick lines)
 *
 * After EB Auton, odometry picks up from where the auton left the robot, so poses are in field
 * coordinates; otherwise it starts at x 0, y 0, heading 0 wherever the robot is.
 *
 * The shooter, the intake, and the Limelight are optional hardware: without a "shooter" or
 * "intake" motor or a "limelight" configured, everything else still works and that hardware's
 * controls do nothing.
 */

@TeleOp(name = "EB Teleop", group = "EBBiobuzz")
public class BiobuzzTeleop extends LinearOpMode {
    private static final double DRIVE_HIGH_POWER = 1.0;
    private static final double DRIVE_LOW_POWER = 0.4;

    // Static so a value tuned with the d-pad carries over to the next run, until the robot
    // app restarts
    private static double shooterPower = 0.5;

    private Follower follower;
    private FollowerTelemetry followerTelemetry;
    private Mecanum drivetrain;
    private Shooter shooter;
    private Intake intake;
    private HiveAim hiveAim;
    private final LogLevel logLevel = new LogLevel();
    private boolean fastDriveMode = true;
    private boolean redAlliance = true;
    // Where redAlliance and the starting pose came from, for telemetry
    private String allianceSource = "default";
    private String poseSource = "started at 0, 0";

    @Override
    public void runOpMode() {
        initHardware();

        // Wait for the game to start (driver presses START). Alliance can be chosen meanwhile.
        while (opModeInInit()) {
            toggleAlliance();
            logLevel.update(gamepad1, gamepad2);
            follower.update();  // keeps odometry current, for remembering where the HIVE is
            hiveAim.update(redAlliance, false);
            addStatusTelemetry();
            telemetry.addData(">", "Press gamepad2 B to change alliance. Press START.");
            logLevel.addTelemetry(telemetry);
            telemetry.update();
        }
        if (isStopRequested()) {
            hiveAim.stop();
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
            hiveAim.update(redAlliance, gamepad1.y);
            drive();
            shoot();
            runIntake();
            updateTelemetry();
        }

        hiveAim.stop();
    }

    private void initHardware() {
        follower = Constants.createFollower(hardwareMap);
        takeMatchSettings();
        followerTelemetry = new FollowerTelemetry(follower, telemetry);
        drivetrain = (Mecanum) follower.getDrivetrain();
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        hiveAim = new HiveAim(hardwareMap, follower);

        // Started during init so the driver can check that the tags are visible before START
        hiveAim.start();
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

    private void toggleAlliance() {
        // Read every pass, so a press made while Start is held is used up rather than counted
        // once Start is let go
        boolean bPressed = gamepad2.bWasPressed();
        if (bPressed && !gamepad2.start) {
            redAlliance = !redAlliance;
            allianceSource = "set with B";
        }
    }

    private void drive() {
        // Check if FastMode is being toggled on or off
        if (gamepad1.a) {
            fastDriveMode = true;
        } else if (gamepad1.b) {
            fastDriveMode = false;
        }

        // Run wheels in POV mode
        // The left stick moves the robot fwd/back and strafes left/right
        // The triggers turn the robot counterclockwise and clockwise
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.left_trigger - gamepad1.right_trigger;

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
        follower.setTeleOpDrive(drive, strafe, turn, true);
        follower.update();
    }

    private void shoot() {
        if (!shooter.isPresent()) {
            return;
        }
        shooterPower = Utils.tuneConstant(telemetry, "Shooter Power (Max)", shooterPower,
                gamepad2.dpadUpWasPressed(), gamepad2.dpadDownWasPressed(), 0.05, 1.0);
        shooter.setPower(gamepad2.right_trigger > 0.25 ? shooterPower : 0);
    }

    /**
     * Each bumper turns its direction on with one press and off with the next; pressing one while
     * the other direction is running switches straight over.
     */
    private void runIntake() {
        boolean forwardPressed = gamepad2.rightBumperWasPressed();
        boolean reversePressed = gamepad2.leftBumperWasPressed();
        Intake.State state = intake.getState();
        if (forwardPressed) {
            intake.set(state == Intake.State.FORWARD ? Intake.State.OFF : Intake.State.FORWARD);
        } else if (reversePressed) {
            intake.set(state == Intake.State.REVERSE ? Intake.State.OFF : Intake.State.REVERSE);
        }
    }

    /** Pinpoint, shooter, intake, and aim status, shown both during init and while driving. */
    private void addStatusTelemetry() {
        telemetry.addData("Pinpoint", followerTelemetry.pinpointStatus());
        telemetry.addData("Pose", "%s (%s)",
                FollowerTelemetry.describePose(follower.getPose()), poseSource);
        if (shooter.isPresent()) {
            telemetry.addData("Shooter RPM", "%.0f", shooter.getRpm());
        } else {
            telemetry.addData("Shooter", "not found -- shooter controls disabled");
        }
        telemetry.addData("Intake", intake.isPresent()
                ? String.valueOf(intake.getState())
                : "not found -- intake controls disabled");
        telemetry.addData("Aim", "%s (%s), %s", redAlliance ? "RED" : "BLUE", allianceSource,
                hiveAim.describeStatus());
        telemetry.addData("HIVE", hiveAim.describeHive());
        if (logLevel.isDebug()) {
            hiveAim.addTagTelemetry(telemetry);
        }
    }

    private void updateTelemetry() {
        addStatusTelemetry();
        telemetry.addData("Fast Drive Mode", fastDriveMode);

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

        logLevel.addTelemetry(telemetry);
        telemetry.update();
    }
}
