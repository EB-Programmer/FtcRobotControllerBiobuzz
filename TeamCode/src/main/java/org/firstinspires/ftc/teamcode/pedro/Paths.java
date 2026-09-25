package org.firstinspires.ftc.teamcode.pedro;

// To run a new route: paste the body the online path visualizer generates -- its imports,
// fields, and path1()/path2()/... methods -- over everything below this package line, then
// delete whatever was here before. BiobuzzAuton runs path1(), path2(), ... in order and stops
// at the first missing number, so it doesn't matter how many paths the new route has.

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(0, 0, 0);
    private final Pose path1 = poseFactory.of(24, 0, 0);

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }
}
