package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;
import java.util.List;

/**
 * The visible tags from one CELL's AprilTag Cluster: four tags in a row on a single sticker, with
 * IDs increasing left to right as seen when the CELL is up. With two or more of them in view,
 * the order they appear in tells whether the CELL is up or down, and their known spacing locates
 * the middle of the cluster even when only some of its tags are visible.
 */
final class TagCluster {
    private static final int FIRST_ID = 30;
    private static final int TAGS_PER_CLUSTER = 4;
    // Each tag's offset from the cluster's middle along the row, in inches, by position in the
    // cluster (lowest ID first), left to right as seen when the CELL is up
    private static final double[] OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};
    // Below this many degrees of tx per inch along the row, the row is seen too nearly end-on to
    // tell which way it runs
    private static final double MIN_ORDER_SLOPE = 0.1;

    enum Orientation { UP, DOWN, UNKNOWN }

    /** Which cluster a HIVE tag ID belongs to: 0 for 30-33, 1 for 34-37, and so on. */
    static int clusterIndex(int tagId) {
        return (tagId - FIRST_ID) / TAGS_PER_CLUSTER;
    }

    private final int index;
    private final int firstId;
    private final List<LLResultTypes.FiducialResult> tags = new ArrayList<>();
    private final List<Pose> fieldPositions = new ArrayList<>();

    TagCluster(int index) {
        this.index = index;
        firstId = FIRST_ID + index * TAGS_PER_CLUSTER;
    }

    int index() {
        return index;
    }

    /** Adds a visible tag, with its field position, or null if it has no 3D position. */
    void add(LLResultTypes.FiducialResult tag, Pose fieldPosition) {
        tags.add(tag);
        fieldPositions.add(fieldPosition);
    }

    List<LLResultTypes.FiducialResult> getTags() {
        return tags;
    }

    int size() {
        return tags.size();
    }

    /** The cluster's ID range, e.g. "30-33". */
    String label() {
        return firstId + "-" + (firstId + TAGS_PER_CLUSTER - 1);
    }

    /** Average tag area in the image; bigger means nearer. */
    double meanArea() {
        double sum = 0;
        for (LLResultTypes.FiducialResult tag : tags) {
            sum += tag.getTargetArea();
        }
        return sum / tags.size();
    }

    /** UP or DOWN from the left-to-right order of the visible tags; UNKNOWN with fewer than two. */
    Orientation orientation() {
        if (tags.size() < 2) {
            return Orientation.UNKNOWN;
        }
        double slope = fitTx()[1];
        if (slope > MIN_ORDER_SLOPE) {
            return Orientation.UP;
        } else if (slope < -MIN_ORDER_SLOPE) {
            return Orientation.DOWN;
        }
        return Orientation.UNKNOWN;
    }

    /**
     * Horizontal angle to the middle of the cluster, from the visible tags' angles and their known
     * offsets along the row. With only one tag visible, the angle to that tag.
     */
    double middleTx() {
        return fitTx()[0];
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
        return new Pose(fitLine(offsets, xs)[0], fitLine(offsets, ys)[0]);
    }

    /** {tx at the middle, tx change per inch along the row} for the visible tags. */
    private double[] fitTx() {
        List<Double> offsets = new ArrayList<>();
        List<Double> txs = new ArrayList<>();
        for (LLResultTypes.FiducialResult tag : tags) {
            offsets.add(offsetOf(tag));
            txs.add(tag.getTargetXDegrees());
        }
        return fitLine(offsets, txs);
    }

    private double offsetOf(LLResultTypes.FiducialResult tag) {
        return OFFSETS_IN[tag.getFiducialId() - firstId];
    }

    /**
     * Least-squares line through (x, y) points: {y at x = 0, slope}. With a single point, or all
     * points at the same x, the slope is 0 and the line runs through their average y.
     */
    private static double[] fitLine(List<Double> xs, List<Double> ys) {
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
        return new double[]{meanY - slope * meanX, slope};
    }
}
