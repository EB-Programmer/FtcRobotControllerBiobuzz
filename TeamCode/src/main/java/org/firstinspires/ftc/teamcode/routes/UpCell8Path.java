package org.firstinspires.ftc.teamcode.routes;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class UpCell8Path {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.7, 9, 90);
    private final Pose path1 = poseFactory.of(58, 34, 90);
    private final Pose point2Start = poseFactory.of(58, 34, 90);
    private final Pose point2 = poseFactory.of(30, 9.2, 180);
    private final Pose point3Start = poseFactory.of(30, 9.2, 180);
    private final Pose point3 = poseFactory.of(9.2, 9.2, 180);
    private final Pose point4Start = poseFactory.of(9.2, 9.2, 180);
    private final Pose point4 = poseFactory.of(58, 34, 90);
    private final Pose point5Start = poseFactory.of(58, 34, 90);
    private final Pose point5 = poseFactory.of(60, 42, 270);
    private final Pose point6Start = poseFactory.of(60, 42, 270);
    private final Pose point6 = poseFactory.of(59, 107.5, 270);
    private final Pose point7Start = poseFactory.of(59, 107.5, 270);
    private final Pose point7 = poseFactory.of(38, 105, 270);
    private final Pose point8Start = poseFactory.of(38, 105, 270);
    private final Pose point8 = poseFactory.of(13, 92, 270);

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

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }
}
