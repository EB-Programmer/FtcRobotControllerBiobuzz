package org.firstinspires.ftc.teamcode.routes;

public class DownCell4Route extends Route {
    public DownCell4Route() {
        super(new DownCell4Path());
    }

    @Override
    public void afterPath(int pathNumber, RouteActions robot) {
        if (pathNumber == 1) {
            // The shooter stays off until 10 s in, then warms up if the wait is still going
            robot.waitForCellReady(23, 10);
            if (robot.autonSeconds() < 23) {
                robot.shoot();
            }
            robot.stopShooter();
        }
    }
}
