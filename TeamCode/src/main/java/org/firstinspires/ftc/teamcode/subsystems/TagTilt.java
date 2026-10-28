package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Tells standing AprilTags from ones lying nearly flat, using how far the Limelight's 3D pose has
 * the tag tilted back. The up CELL's tags stand up and face the camera, reading close to the
 * camera's own mount tilt (a few degrees). The down CELL's tags lie nearly flat, reading close to
 * 90° minus that. The tilt doesn't depend on how far away the tag is or where it is in the
 * picture, so the two stay far apart from anywhere on the field.
 *
 * Needs Full 3D turned on in the Limelight's AprilTag pipeline (its Advanced tab). Without a 3D
 * pose a tag can't be checked, and counts as standing.
 */
final class TagTilt {
    // A tag tilted back more than this counts as lying flat. Standing tags read a few degrees
    // and flat ones in the 80s, so 45° sits halfway.
    private static final double MAX_STANDING_TILT_DEG = 45.0;

    private TagTilt() {
    }

    static boolean hasTilt(LLResultTypes.FiducialResult tag) {
        return LimelightMount.has3D(tag.getTargetPoseCameraSpace());
    }

    /**
     * How far the tag is tilted toward or away from the camera, in degrees: near 0 facing it, near
     * ±90 lying flat. NaN without a 3D pose.
     */
    static double tiltDeg(LLResultTypes.FiducialResult tag) {
        if (!hasTilt(tag)) {
            return Double.NaN;
        }
        // The Limelight's pitch, which the SDK files under roll (see TagDebugTelemetry)
        return tag.getTargetPoseCameraSpace().getOrientation().getRoll(AngleUnit.DEGREES);
    }

    /** Whether the tag's 3D pose shows it lying nearly flat; false without a 3D pose. */
    static boolean isLyingFlat(LLResultTypes.FiducialResult tag) {
        if (!hasTilt(tag)) {
            return false;
        }
        return Math.abs(tiltDeg(tag)) > MAX_STANDING_TILT_DEG;
    }
}
