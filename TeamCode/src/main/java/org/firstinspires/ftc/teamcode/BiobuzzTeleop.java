package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

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

    private Follower follower;
    private Mecanum drivetrain;
    private boolean fastDriveMode = true;

    @Override
    public void runOpMode() {
        initHardware();

        // Wait for the game to start (driver presses START)
        waitForStart();

        follower.startTeleopDrive();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive();
            updateTelemetry();
        }
    }

    private void initHardware() {
        follower = Constants.createFollower(hardwareMap);
        drivetrain = (Mecanum) follower.getDrivetrain();

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
        // The triggers turn the robot counterclockwise and clockwise
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.left_trigger - gamepad1.right_trigger;

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

    private void updateTelemetry() {
        telemetry.addData("Fast Drive Mode", fastDriveMode);

        List<DcMotorEx> motors = drivetrain.getMotors();
        telemetry.addData("Front Left Power", motors.get(0).getPower());
        telemetry.addData("Front Right Power", motors.get(2).getPower());
        telemetry.addData("Rear Left Power", motors.get(1).getPower());
        telemetry.addData("Rear Right Power", motors.get(3).getPower());

        telemetry.addData("gamepad1 LeftStick Y (-drive)", gamepad1.left_stick_y);
        telemetry.addData("gamepad1 LeftStick X (strafe)", gamepad1.left_stick_x);
        telemetry.addData("gamepad1 RightTrigger (turn)", gamepad1.right_trigger);
        telemetry.addData("gamepad1 LeftTrigger (-turn)", gamepad1.left_trigger);

        telemetry.update();
    }
}
