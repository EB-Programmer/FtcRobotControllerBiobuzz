package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.routes.Route;
import org.firstinspires.ftc.teamcode.routes.RouteActions;
import org.firstinspires.ftc.teamcode.routes.UpCellRoute;
import org.firstinspires.ftc.teamcode.util.FollowerTelemetry;
import org.firstinspires.ftc.teamcode.util.InitMenu;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Runs a Route from the routes package: its atStart() actions, then path1(), path2(), ... in
 * order, each followed by its afterPath() actions while the robot holds that path's end pose.
 * During init the driver picks the route (the first one in ROUTES, unless changed) from a
 * settings menu on the d-pad. See pedro/paths for how to add a new route from the online path
 * visualizer.
 */
@Autonomous(name = "EB Auton", group = "EBBiobuzz")
public class BiobuzzAuton extends LinearOpMode {

    /** Every Route the init menu offers, listed by class name. The first is the default. */
    private static final Class<?>[] ROUTES = {
            UpCellRoute.class,
    };

    private static final double MAX_POWER = 0.5;

    private Follower follower;
    private FollowerTelemetry followerTelemetry;
    private Route route;
    private String routeName;
    private List<Path> pathSequence = new ArrayList<>();
    private String routeError;
    private RouteActions routeActions;
    private final ElapsedTime autonTimer = new ElapsedTime();
    private final LoopTimer loopTimer = new LoopTimer();

    /** Which step's actions are running and the pose held meanwhile, for the hold loop. */
    private String actionStep;
    private Pose holdPose;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        followerTelemetry = new FollowerTelemetry(follower, telemetry);
        runInitMenu();
        if (isStopRequested()) return;

        List<String> results = runRoute();
        showSummaryUntilStop(results);
    }

    /** Shows the setup menu until Start (or Stop), loading whichever route is chosen. */
    private void runInitMenu() {
        InitMenu menu = new InitMenu("EB Auton setup");
        InitMenu.Choice<Class<?>> routeChoice =
                menu.choice("Route", ROUTES, Class::getSimpleName);

        while (opModeInInit()) {
            menu.update(gamepad1, gamepad2, telemetry);
            if (routeChoice.changed()) {
                loadRoute(routeChoice.get());
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
     * Builds the chosen route and its paths and moves the follower's starting pose to the
     * route's start. The follower already exists by now, so each path picks up
     * Constants.pathConstraints, which createFollower installs as the default constraints for
     * newly constructed paths.
     *
     * A route that can't be built leaves pathSequence empty, so the robot stays put, and sets
     * routeError so the problem shows on the Driver Station during init instead of crashing.
     */
    private void loadRoute(Class<?> routeClass) {
        routeName = routeClass.getSimpleName();
        route = null;
        pathSequence = new ArrayList<>();
        routeError = null;
        try {
            Object instance = routeClass.getDeclaredConstructor().newInstance();
            if (!(instance instanceof Route)) {
                routeError = routeName + " isn't a Route -- ROUTES should list classes from the"
                        + " routes package, not the pasted paths";
                return;
            }
            List<Path> paths = ((Route) instance).buildPaths();
            if (paths.isEmpty()) {
                routeError = routeName + "'s paths have no path1() method";
                return;
            }
            route = (Route) instance;
            pathSequence = paths;
            follower.setStartingPose(pathSequence.get(0).getPose(0));
        } catch (Exception e) {
            Throwable cause = e instanceof InvocationTargetException ? e.getCause() : e;
            routeError = "Couldn't build " + routeName + ": " + cause;
        }
    }

    /**
     * Runs the route's start actions, then follows every path in order with its afterPath
     * actions, and returns a summary line for each step that finished.
     */
    private List<String> runRoute() {
        autonTimer.reset();
        List<String> results = new ArrayList<>();
        if (route == null) {
            return results;
        }
        routeActions = new RouteActions(this::opModeIsActive, this::holdLoop);

        Pose startPose = pathSequence.get(0).getPose(0);
        follower.holdPoint(startPose);
        String startResult = runActions("start", startPose, () -> route.atStart(routeActions));
        if (!startResult.isEmpty()) {
            results.add("Start: " + startResult);
        }

        for (int i = 0; i < pathSequence.size() && opModeIsActive(); i++) {
            int pathNumber = i + 1;
            String result = runPath(i);
            if (!opModeIsActive()) {
                break;
            }
            Pose endPose = pathSequence.get(i).getPose(1);
            String actionResult = runActions("after path " + pathNumber, endPose,
                    () -> route.afterPath(pathNumber, routeActions));
            results.add(actionResult.isEmpty() ? result : result + "; then " + actionResult);
        }
        follower.breakFollowing();
        return results;
    }

    /** Follows one path until it ends, with live telemetry, and returns its summary line. */
    private String runPath(int pathIndex) {
        Path path = pathSequence.get(pathIndex);
        follower.followPath(new PathChain(path), MAX_POWER, true);
        ElapsedTime pathTimer = new ElapsedTime();
        loopTimer.reset();
        boolean stalled = false;

        while (opModeIsActive() && follower.isBusy()) {
            // Sampled before update(), since the update that ends a stalled path also
            // clears the follower's stuck flag.
            stalled = followerTelemetry.isStalled();
            follower.update();
            addFollowingTelemetry(pathIndex, path, loopTimer.lap());
            telemetry.update();
        }
        return String.format(Locale.US, "Path %d: %.1fs, ended %s%s",
                pathIndex + 1, pathTimer.seconds(),
                followerTelemetry.describeOffset(path.getPose(1)),
                stalled ? " (STALLED)" : "");
    }

    /** Runs one step's route actions while holding holdPose; returns its part of the summary. */
    private String runActions(String step, Pose holdPose, Runnable actions) {
        actionStep = step;
        this.holdPose = holdPose;
        loopTimer.reset();
        return routeActions.run(actions);
    }

    /** One pass of the loop that runs while route actions wait: hold position and report. */
    private void holdLoop(String activity) {
        follower.update();
        double loopMs = loopTimer.lap();

        telemetry.addData("Action", "%s: %s,   %.1fs / 30s total",
                actionStep, activity, autonTimer.seconds());
        followerTelemetry.addPose(holdPose);
        followerTelemetry.addHealth(loopMs);
        telemetry.update();
    }

    private void addFollowingTelemetry(int pathIndex, Path path, double loopMs) {
        telemetry.addData("Path", "%d / %d,   %.1fs / 30s total",
                pathIndex + 1, pathSequence.size(), autonTimer.seconds());
        followerTelemetry.addPose(path.getPose(1));
        //telemetry.addData("Off path", "%.1f in, %.1f° heading",
        //        follower.getTranslationalError().getMagnitude(),
        //        Math.toDegrees(follower.getHeadingError()));
        //telemetry.addData("Progress", "%.0f%%, %.1f in left",
        //        follower.getPathCompletion() * 100, follower.getDistanceRemaining());
        followerTelemetry.addHealth(loopMs);

        if (followerTelemetry.isStalled()) {
            telemetry.addLine("!! STALLED short of the path's end -- path will end early !!");
        }
    }

    /** Shows how the route went, staying running so it remains on screen until Stop is pressed. */
    private void showSummaryUntilStop(List<String> results) {
        if (routeError == null) {
            telemetry.addLine(String.format(Locale.US, "%s finished in %.1fs",
                    routeName, autonTimer.seconds()));
        } else {
            telemetry.addLine("ROUTE ERROR -- robot didn't move");
            telemetry.addLine(routeError);
        }
        for (String result : results) {
            telemetry.addLine(result);
        }
        telemetry.update();
        while (opModeIsActive()) {
            idle();
        }
    }
}
