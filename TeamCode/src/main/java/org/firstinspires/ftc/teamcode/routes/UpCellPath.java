package org.firstinspires.ftc.teamcode.routes;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class UpCellPath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56.237, 28.639, 90);
    private final Pose path2Start = poseFactory.of(56.237, 28.639, 90);
    private final Pose path2 = poseFactory.of(10.991, 45.926, 180);
    private final Pose path3Start = poseFactory.of(10.991, 45.926, 180);
    private final Pose path3 = poseFactory.of(56.213, 28.578, 90);
    private final Pose path4Start = poseFactory.of(56.213, 28.578, 90);
    private final Pose path4 = poseFactory.of(8.253, 8.081, 180);
    private final Pose path5Start = poseFactory.of(8.253, 8.081, 180);
    private final Pose path5 = poseFactory.of(57.091, 28.425, 90);
    private final Pose path6Start = poseFactory.of(57.091, 28.425, 90);
    private final Pose path6 = poseFactory.of(8.333, 95, 90);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path2Start, path2).linear(path2Start, path2);
    }

    public Path path3() {
        return Paths.line(path3Start, path3).linear(path3Start, path3);
    }

    public Path path4() {
        return Paths.line(path4Start, path4).linear(path4Start, path4);
    }

    public Path path5() {
        return Paths.line(path5Start, path5).linear(path5Start, path5);
    }

    public Path path6() {
        return Paths.line(path6Start, path6).linear(path6Start, path6);
    }
}
