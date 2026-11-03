package org.firstinspires.ftc.teamcode.routes;

public class UpCell4Route extends Route {
    public UpCell4Route() {
        super(new UpCell4Path());
    }

    @Override
    public void afterPath(int pathNumber, RouteActions robot) {
        switch (pathNumber) {
            case 2: robot.intakeOn(); robot.waitMillis(250); break;
            case 4: robot.waitMillis(1000); break;
            case 5: robot.intakeOff(); robot.waitMillis(250); break;
        }
    }
}
