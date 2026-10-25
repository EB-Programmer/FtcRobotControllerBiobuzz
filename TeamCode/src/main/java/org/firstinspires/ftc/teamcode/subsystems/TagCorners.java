package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

/**
 * Tells upright AprilTags from upside-down ones using the tag's corners in the camera picture.
 * The Limelight lists each tag's corners in the same order around the printed tag, so the edge
 * from corner 0 to corner 1 points right in the picture when the tag is upright, and left when
 * it's upside down.
 *
 * Needs corner output turned on in the Limelight's AprilTag pipeline (its Output tab). Without
 * corners a tag can't be checked, and counts as upright.
 */
final class TagCorners {
    // An edge pointing within this many degrees of straight right in the picture counts as
    // upright. Upright tags read near 0° and upside-down ones near ±180°, so 90° sits halfway.
    private static final double MAX_UPRIGHT_EDGE_DEG = 90.0;

    private TagCorners() {
    }

    static boolean hasCorners(LLResultTypes.FiducialResult tag) {
        return tag.getTargetCorners().size() >= 2;
    }

    /**
     * Which way the edge from corner 0 to corner 1 points in the picture, in degrees: 0 is right,
     * ±180 is left, and positive is clockwise on screen (the picture's y runs downward). NaN
     * without corners.
     */
    static double edgeAngleDeg(LLResultTypes.FiducialResult tag) {
        if (!hasCorners(tag)) {
            return Double.NaN;
        }
        List<List<Double>> corners = tag.getTargetCorners();
        double dx = corners.get(1).get(0) - corners.get(0).get(0);
        double dy = corners.get(1).get(1) - corners.get(0).get(1);
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    /** Whether the tag's corners show it upside down; false without corners. */
    static boolean isUpsideDown(LLResultTypes.FiducialResult tag) {
        if (!hasCorners(tag)) {
            return false;
        }
        return Math.abs(edgeAngleDeg(tag)) > MAX_UPRIGHT_EDGE_DEG;
    }
}
