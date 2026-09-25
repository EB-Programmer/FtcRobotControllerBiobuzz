package com.pedropathing.api;

import com.pedropathing.math.Pose;

/**
 * Stand-in for Pedro Pathing 3.0's com.pedropathing.api.PoseFactory, which doesn't exist in
 * the 2.0.2 library this project depends on. See Paths for the rest of this compatibility
 * layer. Only degrees()/radians()/of(...) are implemented -- that's what the online path
 * visualizer's generated Paths.java actually uses.
 */
public final class PoseFactory {
    private final boolean useDegrees;

    private PoseFactory(boolean useDegrees) {
        this.useDegrees = useDegrees;
    }

    public static PoseFactory degrees() {
        return new PoseFactory(true);
    }

    public static PoseFactory radians() {
        return new PoseFactory(false);
    }

    public Pose of(double x, double y, double heading) {
        return new Pose(x, y, useDegrees ? Math.toRadians(heading) : heading);
    }
}
