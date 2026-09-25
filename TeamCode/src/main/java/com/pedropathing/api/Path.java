package com.pedropathing.api;

import com.pedropathing.geometry.Curve;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.HeadingInterpolator;

/**
 * Stand-in for Pedro Pathing 3.0's fluent com.pedropathing.paths.Path additions (linear,
 * constant, tangent, reverseTangent, facingPoint), which don't exist in the 2.0.2 library
 * this project depends on. This extends the real 2.0.2 com.pedropathing.paths.Path -- see
 * Paths for how these get constructed -- so code pasted from the online path visualizer
 * (which generates against 3.0's API) compiles unchanged: a method declared to return the
 * real com.pedropathing.paths.Path can return this subclass directly.
 */
public class Path extends com.pedropathing.paths.Path {
    Path(Curve curve) {
        super(curve);
    }

    public Path linear(Pose from, Pose to) {
        setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
        return this;
    }

    public Path linear(double fromHeadingRadians, double toHeadingRadians) {
        setLinearHeadingInterpolation(fromHeadingRadians, toHeadingRadians);
        return this;
    }

    public Path constant(Pose heading) {
        setConstantHeadingInterpolation(heading.getHeading());
        return this;
    }

    public Path constant(double headingRadians) {
        setConstantHeadingInterpolation(headingRadians);
        return this;
    }

    public Path tangent() {
        setTangentHeadingInterpolation();
        return this;
    }

    public Path reverseTangent() {
        setTangentHeadingInterpolation();
        reverseHeadingInterpolation();
        return this;
    }

    public Path facingPoint(double x, double y) {
        setHeadingInterpolation(HeadingInterpolator.facingPoint(x, y));
        return this;
    }
}
