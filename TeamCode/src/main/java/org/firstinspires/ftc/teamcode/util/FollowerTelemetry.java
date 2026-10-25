package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/** Telemetry lines and status checks describing a Pedro Pathing follower and its localizer. */
public class FollowerTelemetry {
    private final Follower follower;
    private final Telemetry telemetry;
    private final GoBildaPinpointDriver pinpoint;

    public FollowerTelemetry(Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
        Localizer localizer = follower.getPoseTracker().getLocalizer();
        pinpoint = localizer instanceof PinpointLocalizer
                ? ((PinpointLocalizer) localizer).getPinpoint()
                : null;
    }

    /**
     * The Pinpoint's status as of the last follower.update(), e.g. READY or
     * FAULT_NO_PODS_DETECTED -- worth showing during init, since a Pinpoint that's configured but
     * unplugged or faulted doesn't stop an OpMode from starting.
     */
    public String pinpointStatus() {
        return pinpoint == null ? "n/a" : String.valueOf(pinpoint.getDeviceStatus());
    }

    /** Adds the robot's current pose and the goal pose it's driving toward or holding. */
    public void addPose(Pose goal) {
        Pose pose = follower.getPose();
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°",
                pose.getX(), pose.getY(), Utils.degrees(pose.getHeading()));
        telemetry.addData("Goal", "x %.1f  y %.1f  h %.1f°",
                goal.getX(), goal.getY(), Utils.degrees(goal.getHeading()));
    }

    /** Adds loop time and Pinpoint status. */
    public void addHealth(double loopMs) {
        telemetry.addData("Health", "loop %.0f ms, Pinpoint %s", loopMs, pinpointStatus());
    }

    /**
     * Adds a warning if the pose has gone NaN, and nothing otherwise. Add it at the bottom of the
     * screen, so the warning appearing doesn't push other lines down.
     */
    public void addWarnings() {
        if (follower.isLocalizationNAN()) {
            telemetry.addLine("!! POSE IS NaN -- position can't be trusted !!");
        }
    }

    /**
     * Whether the robot has nearly stopped before reaching the end of the path. The follower flags
     * "stuck" whenever the robot is under 1 in/s in the last 20% of a path -- which also happens
     * while braking normally at the end -- and after 500 ms of that it ends the path as if it were
     * done. Only the cases short of the path's end mean the robot didn't get there.
     */
    public boolean isStalled() {
        return follower.isRobotStuck() && !follower.atParametricEnd();
    }

    /** How far the robot's current pose is from goal, e.g. "0.6 in / 2.1° off". */
    public String describeOffset(Pose goal) {
        Pose pose = follower.getPose();
        return String.format(Locale.US, "%.1f in / %.1f° off",
                pose.distanceFrom(goal),
                Math.toDegrees(MathFunctions.getSmallestAngleDifference(
                        pose.getHeading(), goal.getHeading())));
    }
}
