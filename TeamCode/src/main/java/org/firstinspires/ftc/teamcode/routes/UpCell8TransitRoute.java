package org.firstinspires.ftc.teamcode.routes;

public class UpCell8TransitRoute extends Route {
    public UpCell8TransitRoute() {
        super(new UpCell8TransitPath());
    }

    @Override
    public void atStart(RouteActions robot) {
        robot.warmUpShooter();
    }

    @Override
    public void afterPath(int pathNumber, RouteActions robot) {
        switch (pathNumber) {
            case 1: robot.shoot(); break;
            case 2: robot.intakeOn(); robot.waitMillis(250); break;
            case 4: robot.warmUpShooter(); robot.waitMillis(1000); break;
            case 5:
                robot.intakeOff();
                robot.shoot();
                // Path 6 crosses to the DownCell side, so stay clear of our alliance partner there
                // until this time
                robot.waitUntilAutonSeconds(16);
                break;
            case 6: robot.warmUpShooter(); break;
            case 7:
                // Path 7 turns 175° on the way to the DownCell spot, and can end before the turn
                // is done
                robot.waitForHeadingMillis(1500);
                robot.waitForCellReady(23);
                robot.shoot();
                break;
        }
    }
}
