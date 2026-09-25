package com.pedropathing.api;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.math.Pose;

/**
 * Stand-in for Pedro Pathing 3.0's com.pedropathing.api.Paths, which doesn't exist in the
 * 2.0.2 library this project depends on. See PoseFactory and Path for the rest of this
 * compatibility layer. Only line(...) and curve(...) are implemented -- that's what the
 * online path visualizer's generated Paths.java actually uses; path(...)/through(...) and
 * the Vector2D overloads are 3.0-only concepts with no direct 2.0.2 equivalent.
 */
public final class Paths {
    private Paths() {}

    public static Path line(Pose from, Pose to) {
        return new Path(new BezierLine(from.toGeometryPose(), to.toGeometryPose()));
    }

    public static Path curve(Pose... controlPoints) {
        FuturePose[] geometryControlPoints = new FuturePose[controlPoints.length];
        for (int i = 0; i < controlPoints.length; i++) {
            geometryControlPoints[i] = controlPoints[i].toGeometryPose();
        }
        return new Path(new BezierCurve(geometryControlPoints));
    }
}
