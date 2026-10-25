package org.firstinspires.ftc.teamcode.routes;

import org.firstinspires.ftc.teamcode.pedro.paths.UpCellPath;

public class UpCellRoute extends Route {
    public UpCellRoute() {
        super(new UpCellPath());
    }

    @Override
    public void afterPath(int pathNumber, RouteActions robot) {
        robot.waitMillis(500);
    }
}
