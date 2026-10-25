package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Where the Limelight is mounted on the robot, and the math for turning what it sees into field
 * positions. Update these constants if the camera moves.
 */
final class LimelightMount {
    static final double CAMERA_FORWARD_IN = 6.0;  // ahead of the robot's center
    static final double CAMERA_LEFT_IN = 0.0;  // left of the robot's center
    static final double CAMERA_TILT_DEG = 45.0;  // upward from horizontal

    private LimelightMount() {
    }

    /**
     * Where on the field a tag is (x and y, in inches), given its position relative to the camera
     * and the robot's pose when the frame was captured. Null without a 3D position for the tag.
     */
    static Pose tagFieldPosition(Pose3D tagInCamera, Pose robot) {
        if (robot == null || !has3D(tagInCamera)) {
            return null;
        }
        // Limelight camera space: +x right, +y down, +z straight out of the lens
        Position p = tagInCamera.getPosition().toUnit(DistanceUnit.INCH);
        double tilt = Math.toRadians(CAMERA_TILT_DEG);
        double forward = p.z * Math.cos(tilt) + p.y * Math.sin(tilt) + CAMERA_FORWARD_IN;
        double left = -p.x + CAMERA_LEFT_IN;

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
