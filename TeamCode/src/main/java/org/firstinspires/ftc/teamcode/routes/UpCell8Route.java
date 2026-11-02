package org.firstinspires.ftc.teamcode.routes;

public class UpCell8Route extends Route {
    public UpCell8Route() {
        super(new UpCell8Path());
    }

    @Override
    public void afterPath(int pathNumber, RouteActions robot) {
        switch (pathNumber) {
            case 2: robot.intakeOn(); robot.waitMillis(250); break;
            case 3: robot.waitMillis(1000); break;
            case 4: robot.intakeOff(); robot.waitMillis(250); break;
            case 5: robot.waitForHeadingMillis(1000); break;
            case 6: robot.waitForCellReady(26); break;
        }
    }
}
