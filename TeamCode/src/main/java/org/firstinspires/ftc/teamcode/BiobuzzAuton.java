package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Paths;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

/**
 * Runs whatever route is pasted into pedro/Paths.java: path1(), path2(), ... in order, each
 * held at its end pose until the next one starts. See pedro/Paths.java for how to swap in a
 * new route from the online path visualizer.
 */
@Autonomous(name = "EB Auton", group = "EBBiobuzz")
public class BiobuzzAuton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        List<Path> pathSequence = discoverPaths(new Paths());

        Follower follower = Constants.createFollower(hardwareMap);
        if (!pathSequence.isEmpty()) {
            follower.setStartingPose(pathSequence.get(0).getPose(0));
        }

        waitForStart();
        if (isStopRequested()) return;

        for (int i = 0; i < pathSequence.size() && opModeIsActive(); i++) {
            follower.followPath(pathSequence.get(i), true);

            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                telemetry.addData("Path", (i + 1) + " / " + pathSequence.size());
                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.update();
            }
        }

        follower.breakFollowing();
    }

    /**
     * Collects path1(), path2(), ... from the pasted Paths class by the number in each method's
     * name, stopping at the first missing one -- that's the naming convention the online path
     * visualizer uses, so any pasted route runs without its path count being wired in by hand.
     */
    private static List<Path> discoverPaths(Paths paths) {
        List<Path> result = new ArrayList<>();
        for (int i = 1; ; i++) {
            try {
                Method method = Paths.class.getMethod("path" + i);
                result.add((Path) method.invoke(paths));
            } catch (ReflectiveOperationException e) {
                break;
            }
        }
        return result;
    }
}
