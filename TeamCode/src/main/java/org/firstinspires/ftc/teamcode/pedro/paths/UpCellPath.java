package org.firstinspires.ftc.teamcode.pedro.paths;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class UpCellPath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.000, 8.000, 90);
    private final Pose path1 = poseFactory.of(56.237, 28.639, 90);
    private final Pose path2 = poseFactory.of(10.991, 45.926, 180);
    private final Pose path3 = poseFactory.of(56.213, 28.578, 90);
    private final Pose path4 = poseFactory.of(8.253, 8.081, 180);
    private final Pose path5 = poseFactory.of(57.091, 28.425, 90);
    private final Pose path6 = poseFactory.of(8.333, 95.000, 90);

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return line(path1, path2).linear(path1, path2);
    }

    public Path path3() {
        return line(path2, path3).linear(path2, path3);
    }

    public Path path4() {
        return line(path3, path4).linear(path3, path4);
    }

    public Path path5() {
        return line(path4, path5).linear(path4, path5);
    }

    public Path path6() {
        return line(path5, path6).linear(path5, path6);
    }
}
