package com.pedropathing.math;

/**
 * Stand-in for Pedro Pathing 3.0's com.pedropathing.math.Pose, which doesn't exist in the
 * 2.0.2 library this project depends on. This lets Paths.java pasted from the online path
 * visualizer (which generates against 3.0's API) compile unchanged. See PoseFactory and Paths
 * in com.pedropathing.api for the rest of this compatibility layer.
 */
public final class Pose {
    private final double x;
    private final double y;
    private final double headingRadians;

    public Pose(double x, double y, double headingRadians) {
        this.x = x;
        this.y = y;
        this.headingRadians = headingRadians;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return headingRadians;
    }

    public com.pedropathing.geometry.Pose toGeometryPose() {
        return new com.pedropathing.geometry.Pose(x, y, headingRadians);
    }
}
