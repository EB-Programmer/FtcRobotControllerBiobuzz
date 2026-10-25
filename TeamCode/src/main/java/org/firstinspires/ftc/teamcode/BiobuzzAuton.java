package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.paths.UpCellPath;
import org.firstinspires.ftc.teamcode.util.InitMenu;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

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
    private GoBildaPinpointDriver pinpoint;
    private String routeName;
    private List<Path> pathSequence = new ArrayList<>();
    private String routeError;
    private final ElapsedTime autonTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initFollower();
        runInitMenu();
        if (isStopRequested()) return;

        List<String> pathResults = runRoute();
        showSummaryUntilStop(pathResults);
    }

    private void initFollower() {
        follower = Constants.createFollower(hardwareMap);
        Localizer localizer = follower.getPoseTracker().getLocalizer();
        if (localizer instanceof PinpointLocalizer) {
            pinpoint = ((PinpointLocalizer) localizer).getPinpoint();
        }
    }

    /** Shows the setup menu until Start (or Stop), loading whichever route is chosen. */
    private void runInitMenu() {
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
        routeName = route.getSimpleName();
        pathSequence = new ArrayList<>();
        routeError = null;
        try {
            List<Path> paths = discoverPaths(route.getDeclaredConstructor().newInstance());
            if (paths.isEmpty()) {
                routeError = routeName + " has no path1() method";
                return;
            }
            pathSequence = paths;
            follower.setStartingPose(pathSequence.get(0).getPose(0));
        } catch (Exception e) {
            Throwable cause = e instanceof InvocationTargetException ? e.getCause() : e;
            routeError = "Couldn't build " + routeName + ": " + cause;
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

    /** Follows every path in order and returns a summary line for each one that finished. */
    private List<String> runRoute() {
        autonTimer.reset();
        List<String> pathResults = new ArrayList<>();
        for (int i = 0; i < pathSequence.size() && opModeIsActive(); i++) {
            String result = runPath(i);
            if (opModeIsActive()) {
                pathResults.add(result);
            }
        }
        follower.breakFollowing();
        return pathResults;
    }

    /** Follows one path until it ends, with live telemetry, and returns its summary line. */
    private String runPath(int pathIndex) {
        Path path = pathSequence.get(pathIndex);
        follower.followPath(new PathChain(path), MAX_POWER, true);
        ElapsedTime pathTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        boolean stalled = false;

        while (opModeIsActive() && follower.isBusy()) {
            // Sampled before update(), since the update that ends a stalled path also
            // clears the follower's stuck flag.
            stalled = isStalled();
            follower.update();
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            addFollowingTelemetry(pathIndex, path, loopMs);
            telemetry.update();
        }
        return describePathEnd(pathIndex, path, pathTimer.seconds(), stalled);
    }

    private void addFollowingTelemetry(int pathIndex, Path path, double loopMs) {
        Pose pose = follower.getPose();
        Pose goal = path.getPose(1);
        telemetry.addData("Path", "%d / %d,   %.1fs / 30s total",
                pathIndex + 1, pathSequence.size(), autonTimer.seconds());
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°",
                pose.getX(), pose.getY(), degrees(pose.getHeading()));
        telemetry.addData("Goal", "x %.1f  y %.1f  h %.1f°",
                goal.getX(), goal.getY(), degrees(goal.getHeading()));
        //telemetry.addData("Off path", "%.1f in, %.1f° heading",
        //        follower.getTranslationalError().getMagnitude(),
        //        Math.toDegrees(follower.getHeadingError()));
        //telemetry.addData("Progress", "%.0f%%, %.1f in left",
        //        follower.getPathCompletion() * 100, follower.getDistanceRemaining());
        telemetry.addData("Health", "loop %.0f ms, Pinpoint %s",
                loopMs, pinpoint == null ? "n/a" : pinpoint.getDeviceStatus());

        // Conditionally add warnings to telemetry
        if (follower.isLocalizationNAN()) {
            telemetry.addLine("!! POSE IS NaN -- position can't be trusted !!");
        }
        if (isStalled()) {
            telemetry.addLine("!! STALLED short of the path's end -- path will end early !!");
        }
    }

    /**
     * Whether the robot has nearly stopped before reaching the end of the path. The follower flags
     * "stuck" whenever the robot is under 1 in/s in the last 20% of a path -- which also happens
     * while braking normally at the end -- and after 500 ms of that it ends the path as if it were
     * done. Only the cases short of the path's end mean the robot didn't get there.
     */
    private boolean isStalled() {
        return follower.isRobotStuck() && !follower.atParametricEnd();
    }

    /** One summary line for a finished path: how long it took and how far off its goal it ended. */
    private String describePathEnd(int pathIndex, Path path, double pathSeconds, boolean stalled) {
        Pose pose = follower.getPose();
        Pose goal = path.getPose(1);
        return String.format(Locale.US, "Path %d: %.1fs, ended %.1f in / %.1f° off%s",
                pathIndex + 1, pathSeconds, pose.distanceFrom(goal),
                Math.toDegrees(MathFunctions.getSmallestAngleDifference(
                        pose.getHeading(), goal.getHeading())),
                stalled ? " (STALLED)" : "");
    }

    /** Shows how the route went, staying running so it remains on screen until Stop is pressed. */
    private void showSummaryUntilStop(List<String> pathResults) {
        if (routeError == null) {
            telemetry.addLine(String.format(Locale.US, "%s finished in %.1fs",
                    routeName, autonTimer.seconds()));
        } else {
            telemetry.addLine("ROUTE ERROR -- robot didn't move");
            telemetry.addLine(routeError);
        }
        for (String result : pathResults) {
            telemetry.addLine(result);
        }
        telemetry.update();
        while (opModeIsActive()) {
            idle();
        }
    }

    /** Heading in degrees, normalized to [0, 360). */
    private static double degrees(double radians) {
        return Math.toDegrees(MathFunctions.normalizeAngle(radians));
    }
}
