package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Where on the field the HIVE was last seen. The HIVE never moves (TIPs only change which CELL
 * faces up), so this never expires on its own; HiveAim clears it when the alliance changes.
 */
final class HiveMemory {
    // How much each new sighting moves the remembered position (0-1); lower smooths more
    private static final double SMOOTHING = 0.2;

    private Pose position = null;
    private final ElapsedTime sinceSeen = new ElapsedTime();

    boolean isEmpty() {
        return position == null;
    }

    void clear() {
        position = null;
    }

    /** Blends a new sighting of the HIVE at seenAt into the remembered position. */
    void remember(Pose seenAt) {
        if (position == null) {
            position = new Pose(seenAt.getX(), seenAt.getY());
        } else {
            position = new Pose(
                    position.getX() + SMOOTHING * (seenAt.getX() - position.getX()),
                    position.getY() + SMOOTHING * (seenAt.getY() - position.getY()));
        }
        sinceSeen.reset();
    }

    /** The remembered position; only meaningful when not isEmpty(). */
    Pose getPosition() {
        return position;
    }

    double secondsSinceSeen() {
        return sinceSeen.seconds();
    }

    /** Signed degrees from robot's heading to the remembered position, in (-180, 180]. */
    double headingErrorDeg(Pose robot) {
        double bearing = Math.atan2(position.getY() - robot.getY(),
                position.getX() - robot.getX());
        double error = bearing - robot.getHeading();
        return Math.toDegrees(Math.atan2(Math.sin(error), Math.cos(error)));
    }
}
