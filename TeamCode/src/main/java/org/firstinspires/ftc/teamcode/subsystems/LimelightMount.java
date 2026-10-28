package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Where the Limelight is mounted on the robot, and the math for turning what it sees into
 * positions relative to the robot's center and on the field. Update these constants if the camera
 * moves.
 *
 * The camera's height (13 in above the floor) isn't needed: the Limelight's 3D pose already
 * measures how far away each tag is, so nothing here works distance out from height.
 */
final class LimelightMount {
    static final double CAMERA_FORWARD_IN = 5.0;  // ahead of the robot's center
    static final double CAMERA_LEFT_IN = -2.0;  // left of the robot's center (negative is right)
    static final double CAMERA_TILT_DEG = 45.0;  // upward from horizontal

    private LimelightMount() {
    }

    /**
     * Where a tag is relative to the robot's center, in inches, from its position relative to the
     * camera: x is how far ahead and y how far to the left. Null without a 3D position for the
     * tag.
     */
    static Pose tagRobotPosition(Pose3D tagInCamera) {
        if (!has3D(tagInCamera)) {
            return null;
        }
        // Limelight camera space: +x right, +y down, +z straight out of the lens
        Position p = tagInCamera.getPosition().toUnit(DistanceUnit.INCH);
        double tilt = Math.toRadians(CAMERA_TILT_DEG);
        double forward = p.z * Math.cos(tilt) + p.y * Math.sin(tilt) + CAMERA_FORWARD_IN;
        double left = -p.x + CAMERA_LEFT_IN;
        return new Pose(forward, left);
    }

    /**
     * Where on the field (x and y, in inches) a spot is, given where it is relative to the robot's
     * center (see tagRobotPosition) and the robot's pose at the time.
     */
    static Pose fieldPosition(Pose robotRelative, Pose robot) {
        double forward = robotRelative.getX();
        double left = robotRelative.getY();
        double heading = robot.getHeading();
        return new Pose(
                robot.getX() + forward * Math.cos(heading) - left * Math.sin(heading),
                robot.getY() + forward * Math.sin(heading) + left * Math.cos(heading));
    }

    /** The SDK fills in all zeros when the Limelight sends no 3D pose for a tag. */
    static boolean has3D(Pose3D pose) {
        Position position = pose.getPosition();
        return position.x != 0 || position.y != 0 || position.z != 0;
    }
}
