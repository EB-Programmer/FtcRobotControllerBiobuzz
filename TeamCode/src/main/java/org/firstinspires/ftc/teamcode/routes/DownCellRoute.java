package org.firstinspires.ftc.teamcode.routes;

public class DownCellRoute extends Route {
    public DownCellRoute() {
        super(new DownCellPath());
    }

    @Override
    public void afterPath(int pathNumber, RouteActions robot) {
        if (pathNumber == 1) {
            robot.waitForCellReady(27);
        }
    }
}
