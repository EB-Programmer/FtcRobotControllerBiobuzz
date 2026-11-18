package org.firstinspires.ftc.teamcode.routes;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class DownCell4Path {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.2, 132.5, 90);
    private final Pose path1 = poseFactory.of(53.25, 121.5, 90);
    private final Pose point2Start = poseFactory.of(53.25, 121.5, 90);
    private final Pose point2 = poseFactory.of(13, 121, 90);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }
}
