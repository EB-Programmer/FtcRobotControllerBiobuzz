package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.paths.UpCellPath;
import org.firstinspires.ftc.teamcode.util.InitMenu;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

/**
 * Runs a route from the pedro/paths package: path1(), path2(), ... in order, each held at its
 * end pose until the next one starts. During init the driver picks the route (the first one
 * in ROUTES, unless changed) from a settings menu on the d-pad. See pedro/paths for how to add
 * a new route from the online path visualizer.
 */
@Autonomous(name = "EB Auton", group = "EBBiobuzz")
public class BiobuzzAuton extends LinearOpMode {

    /** Every route the init menu offers, listed by class name. The first is the default. */
    private static final Class<?>[] ROUTES = {
            UpCellPath.class,
    };

    private static final double MAX_POWER = 0.5;

    private Follower follower;
    private List<Path> pathSequence = new ArrayList<>();
    private String routeError;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);

        InitMenu menu = new InitMenu("EB Auton setup");
        InitMenu.Choice<Class<?>> route = menu.choice("Route", ROUTES, Class::getSimpleName);

        while (opModeInInit()) {
            menu.update(gamepad1, gamepad2, telemetry);
            if (route.changed()) {
                loadRoute(route.get());
            }
            if (routeError == null) {
                telemetry.addData("Status", "Ready to run");
            } else {
                telemetry.addData("Status", "ROUTE ERROR -- robot won't move");
                telemetry.addLine(routeError);
            }
            telemetry.update();
        }
        if (isStopRequested()) return;

        for (int i = 0; i < pathSequence.size() && opModeIsActive(); i++) {
            Path path = pathSequence.get(i);
            follower.followPath(new PathChain(path), MAX_POWER, true);

            while (opModeIsActive() && follower.isBusy()) {
                follower.update();

                Pose goalPose = path.getPose(1);
                telemetry.addData("Goal Path", i + 1);
                telemetry.addData("Goal X", goalPose.getX());
                telemetry.addData("Goal Y", goalPose.getY());
                telemetry.addData("Goal Heading", path.getHeadingGoal(1));
                telemetry.addData("Path State", i + 1);
                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading", follower.getPose().getHeading());
                telemetry.addData("Follower isBusy", follower.isBusy());
                telemetry.update();
            }
        }

        follower.breakFollowing();
    }

    /**
     * Builds the chosen route's paths and moves the follower's starting pose to the route's
     * start. The follower already exists by now, so each path picks up Constants.pathConstraints,
     * which createFollower installs as the default constraints for newly constructed paths.
     *
     * A route that can't be built leaves pathSequence empty, so the robot stays put, and sets
     * routeError so the problem shows on the Driver Station during init instead of crashing.
     */
    private void loadRoute(Class<?> route) {
        pathSequence = new ArrayList<>();
        routeError = null;
        try {
            List<Path> paths = discoverPaths(route.getDeclaredConstructor().newInstance());
            if (paths.isEmpty()) {
                routeError = route.getSimpleName() + " has no path1() method";
                return;
            }
            pathSequence = paths;
            follower.setStartingPose(pathSequence.get(0).getPose(0));
        } catch (Exception e) {
            Throwable cause = e instanceof InvocationTargetException ? e.getCause() : e;
            routeError = "Couldn't build " + route.getSimpleName() + ": " + cause;
        }
    }

    /**
     * Collects path1(), path2(), ... from a pasted route class by the number in each method's
     * name, stopping at the first missing one -- that's the naming convention the online path
     * visualizer uses, so any pasted route runs without its path count being wired in by hand.
     */
    private static List<Path> discoverPaths(Object route) throws ReflectiveOperationException {
        List<Path> result = new ArrayList<>();
        for (int i = 1; ; i++) {
            Method method;
            try {
                method = route.getClass().getMethod("path" + i);
            } catch (NoSuchMethodException e) {
                break;
            }
            result.add((Path) method.invoke(route));
        }
        return result;
    }
}
