package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * What EB Auton hands to the EB Teleop that runs after it. Static fields keep their values from
 * one OpMode to the next, until the Robot Controller app restarts; null means EB Auton hasn't set
 * that value since then.
 */
public final class MatchSettings {
    private MatchSettings() {
    }

    /** The alliance chosen in EB Auton's menu: true for red, false for blue. */
    public static Boolean redAlliance = null;

    /**
     * Where the robot was when EB Auton ended, in field coordinates. Teleop takes it once and sets
     * it back to null, since by the next teleop run the robot has likely been moved by hand.
     */
    public static Pose autonEndPose = null;
}
