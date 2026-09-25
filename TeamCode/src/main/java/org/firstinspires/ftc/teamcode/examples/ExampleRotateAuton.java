package org.firstinspires.ftc.teamcode.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Rotates the robot 45 degrees clockwise about its own center, without
 * translating, using the Follower's built-in turnDegrees. Heading increases
 * counterclockwise, so a clockwise turn is expressed as isLeft = false.
 */
@Autonomous(name = "EB Auton: Rotate 45 CW", group = "EBBiobuzzTest")
public class ExampleRotateAuton extends LinearOpMode {
    private static final double TURN_DEGREES_CLOCKWISE = 45.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        waitForStart();
        if (isStopRequested()) return;

        double goalHeadingDeg = Math.toDegrees(follower.getPose().getHeading()) - TURN_DEGREES_CLOCKWISE;

        follower.turnDegrees(TURN_DEGREES_CLOCKWISE, false);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Current heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Target heading (deg)", goalHeadingDeg);
            telemetry.update();
        }

        follower.breakFollowing();
    }
}
