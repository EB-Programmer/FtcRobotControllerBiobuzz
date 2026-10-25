package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;
import java.util.List;

/**
 * The visible tags of the up CELL's AprilTag Cluster: four tags in a row on a single sticker,
 * with IDs increasing left to right. Their known spacing locates the middle of the cluster even
 * when only some of its tags are visible.
 */
final class TagCluster {
    private static final int FIRST_ID = 30;
    private static final int TAGS_PER_CLUSTER = 4;
    // Each tag's offset from the cluster's middle along the row, in inches, by position in the
    // cluster (lowest ID first), left to right as seen when the CELL is up
    private static final double[] OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};

    private final List<LLResultTypes.FiducialResult> tags = new ArrayList<>();
    private final List<Pose> fieldPositions = new ArrayList<>();

    /** Adds a visible tag, with its field position, or null if it has no 3D position. */
    void add(LLResultTypes.FiducialResult tag, Pose fieldPosition) {
        tags.add(tag);
        fieldPositions.add(fieldPosition);
    }

    void clear() {
        tags.clear();
        fieldPositions.clear();
    }

    List<LLResultTypes.FiducialResult> getTags() {
        return tags;
    }

    int size() {
        return tags.size();
    }

    boolean isEmpty() {
        return tags.isEmpty();
    }

    /**
     * Horizontal angle to the middle of the cluster, from the visible tags' angles and their known
     * offsets along the row. With only one tag visible, the angle to that tag.
     */
    double middleTx() {
        List<Double> offsets = new ArrayList<>();
        List<Double> txs = new ArrayList<>();
        for (LLResultTypes.FiducialResult tag : tags) {
            offsets.add(offsetOf(tag));
            txs.add(tag.getTargetXDegrees());
        }
        return fitLineAtZero(offsets, txs);
    }

    /**
     * The middle of the cluster on the field, from the visible tags' field positions and their
     * known offsets along the row. With only one positioned tag, that tag's position; null when
     * no visible tag has a 3D position.
     */
    Pose middleFieldPosition() {
        List<Double> offsets = new ArrayList<>();
        List<Double> xs = new ArrayList<>();
        List<Double> ys = new ArrayList<>();
        for (int i = 0; i < tags.size(); i++) {
            Pose position = fieldPositions.get(i);
            if (position != null) {
                offsets.add(offsetOf(tags.get(i)));
                xs.add(position.getX());
                ys.add(position.getY());
            }
        }
        if (offsets.isEmpty()) {
            return null;
        }
        return new Pose(fitLineAtZero(offsets, xs), fitLineAtZero(offsets, ys));
    }

    /** The tag's offset along the row, from its place in its cluster's IDs (30-33: 0-3, ...). */
    private static double offsetOf(LLResultTypes.FiducialResult tag) {
        return OFFSETS_IN[(tag.getFiducialId() - FIRST_ID) % TAGS_PER_CLUSTER];
    }

    /**
     * Draws the best-fitting straight line through the (x, y) points and returns its y at x = 0
     * -- here, the value at the cluster's middle, which has no tag of its own. With a single
     * point, or all points at the same x, that's just their average y.
     */
    private static double fitLineAtZero(List<Double> xs, List<Double> ys) {
        int n = xs.size();
        double meanX = 0;
        double meanY = 0;
        for (int i = 0; i < n; i++) {
            meanX += xs.get(i) / n;
            meanY += ys.get(i) / n;
        }
        double covariance = 0;
        double variance = 0;
        for (int i = 0; i < n; i++) {
            covariance += (xs.get(i) - meanX) * (ys.get(i) - meanY);
            variance += (xs.get(i) - meanX) * (xs.get(i) - meanX);
        }
        double slope = variance == 0 ? 0 : covariance / variance;
        return meanY - slope * meanX;
    }
}
