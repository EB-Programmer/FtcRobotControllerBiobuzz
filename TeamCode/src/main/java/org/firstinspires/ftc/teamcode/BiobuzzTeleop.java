package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Map;

/*
 * Controls for Gamepad 1:
 *   Left Stick:     Move forward & backward, strafe left & right
 *   Left Trigger:   Turn counterclockwise
 *   Right Trigger:  Turn clockwise
 *   A:              Fast Drive Mode
 *   B:              Slow Drive Mode
 *
 * Controls for Gamepad 2:
 *   Left Bumper:
 *   Right Bumper:
 *   Right Trigger:
 *   A:
 *   B:
 *   Y:
 */

@TeleOp(name = "EB Teleop", group = "EBBiobuzz")
public class BiobuzzTeleop extends LinearOpMode {
    private static final double DRIVE_HIGH_POWER = 1.0;
    private static final double DRIVE_LOW_POWER = 0.4;

    private Mecanum drivetrain;
    private boolean fastDriveMode = true;

    @Override
    public void runOpMode() {
        initHardware();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive();
            updateTelemetry();
        }
    }

    private void initHardware() {
        drivetrain = new Mecanum(hardwareMap, Constants.driveConfig);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();
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
        // The right stick turns the robot counterclockwise and clockwise
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_trigger - gamepad1.left_trigger;

        double powerLimit = fastDriveMode ? DRIVE_HIGH_POWER : DRIVE_LOW_POWER;

        // If turning during Low Power Mode: decrease max speed even more for fine-tune aiming
        if (!fastDriveMode && Math.abs(drive) < 0.01 && Math.abs(strafe) < 0.01) {
            powerLimit /= 2;
        }

        // Mecanum's strafe and turn axes point opposite this stick mapping, so negate them
        // here to keep turning and strafing feel matched to the drive (forward/back) axis.
        DrivePowers powers = new DrivePowers(drive, -strafe, -turn);

        // Scale down (rather than just clip) if this exceeds the current mode's power limit,
        // so slow mode stays proportional instead of clamping to a differently-shaped output.
        double maxWheelPower = 0;
        for (double wheelPower : drivetrain.computeWheelPowersUnnormalized(powers)) {
            maxWheelPower = Math.max(maxWheelPower, Math.abs(wheelPower));
        }
        if (maxWheelPower > powerLimit) {
            double scale = powerLimit / maxWheelPower;
            powers = new DrivePowers(drive * scale, -strafe * scale, -turn * scale);
        }

        drivetrain.drive(powers, true);
    }

    private void updateTelemetry() {
        telemetry.addData("Fast Drive Mode", fastDriveMode);

        Map<String, Object> drivetrainDebug = drivetrain.debug();
        telemetry.addData("Front Left Power", drivetrainDebug.get("leftFrontWheelPower"));
        telemetry.addData("Front Right Power", drivetrainDebug.get("rightFrontWheelPower"));
        telemetry.addData("Rear Left Power", drivetrainDebug.get("leftBackWheelPower"));
        telemetry.addData("Rear Right Power", drivetrainDebug.get("rightBackWheelPower"));

        telemetry.addData("gamepad1 LeftStick Y (-drive)", gamepad1.left_stick_y);
        telemetry.addData("gamepad1 LeftStick X (strafe)", gamepad1.left_stick_x);
        telemetry.addData("gamepad1 RightTrigger (turn)", gamepad1.right_trigger);
        telemetry.addData("gamepad1 LeftTrigger (-turn)", gamepad1.left_trigger);

        telemetry.update();
    }
}
