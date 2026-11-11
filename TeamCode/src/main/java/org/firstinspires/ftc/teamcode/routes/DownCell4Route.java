package org.firstinspires.ftc.teamcode.routes;

public class DownCell4Route extends Route {
    public DownCell4Route() {
        super(new DownCell4Path());
    }

    @Override
    public void atStart(RouteActions robot) {
        robot.warmUpShooter();
    }

    @Override
    public void afterPath(int pathNumber, RouteActions robot) {
        if (pathNumber == 1) {
            robot.waitForCellReady(25);
            if (robot.autonSeconds() < 20) {
                robot.shoot();
            }
        }
    }
}
