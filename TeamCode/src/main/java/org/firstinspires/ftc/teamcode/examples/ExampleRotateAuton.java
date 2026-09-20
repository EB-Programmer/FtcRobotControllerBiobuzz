package org.firstinspires.ftc.teamcode.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Rotates the robot 45 degrees clockwise about its own center, without
 * translating, using the Follower's hold mode. Path curves in this library
 * reject zero-length segments, so an in-place turn can't be expressed as a
 * followed Path -- holding a stationary target pose with a different
 * heading is the mechanism for this instead. Heading increases
 * counterclockwise, so a clockwise turn subtracts from it.
 */
@Autonomous(name = "EB Auton: Rotate 45 CW", group = "EBBiobuzzTest")
public class ExampleRotateAuton extends LinearOpMode {
    private static final double TURN_DEGREES_CLOCKWISE = 45.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setPose(Pose.zero());

        waitForStart();
        if (isStopRequested()) return;

        double goalHeading = follower.pose().heading() - Math.toRadians(TURN_DEGREES_CLOCKWISE);
        rotateTo(follower, goalHeading);

        follower.stop();
    }

    /**
     * Holds the robot's current position while rotating it to goalHeading
     * (radians), blocking until the turn converges or the OpMode is stopped.
     */
    private void rotateTo(Follower follower, double goalHeading) {
        Pose targetPose = follower.pose().withHeading(goalHeading);

        follower.hold(targetPose);
        follower.algorithm().reset(); // arms isBusy() and clears controller state for the new target

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Current heading (deg)", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Target heading (deg)", Math.toDegrees(goalHeading));
            telemetry.update();
        }
    }
}
