package org.firstinspires.ftc.teamcode.routes;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class UpCell4Path {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56, 33, -180);
    private final Pose point2Start = poseFactory.of(56, 33, -180);
    private final Pose point2 = poseFactory.of(33.2304, 7.6879, 180);
    private final Pose point3Start = poseFactory.of(33.2304, 7.6879, 180);
    private final Pose point3 = poseFactory.of(9.1301, 6.0065, 30);
    private final Pose point4Start = poseFactory.of(9.1301, 6.0065, 30);
    private final Pose point4 = poseFactory.of(30.0312, 26.7178, 35);
    private final Pose point5Start = poseFactory.of(30.0312, 26.7178, 0);
    private final Pose point5 = poseFactory.of(10.3286, 94.2304, 0);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }
}
