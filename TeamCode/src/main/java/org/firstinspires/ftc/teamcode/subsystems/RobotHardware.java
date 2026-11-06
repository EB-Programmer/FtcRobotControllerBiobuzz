package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.FollowerTelemetry;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * The robot's hardware, set up the same way for every match OpMode, along with the telemetry that
 * reports during init what was found. The shooter, shooter gates, intake, Limelight, and indicator
 * lights are optional, so a missing one doesn't stop an OpMode from starting; the init telemetry
 * is how the driver finds out.
 */
public class RobotHardware {
    public final Follower follower;
    public final FollowerTelemetry followerTelemetry;
    public final Shooter shooter;
    public final ShooterGates gates;
    public final Intake intake;
    public final HiveAim hiveAim;
    public final IndicatorLights lights;

    /**
     * Sets up all the hardware and starts the Limelight, so it's already sending results at Start
     * and the driver can check during init that tags are visible. poseOnField is whether odometry
     * will be in field coordinates (see HiveAim).
     */
    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry, boolean poseOnField) {
        follower = Constants.createFollower(hardwareMap);
        followerTelemetry = new FollowerTelemetry(follower, telemetry);
        shooter = new Shooter(hardwareMap);
        gates = new ShooterGates(hardwareMap);
        intake = new Intake(hardwareMap);
        hiveAim = new HiveAim(hardwareMap, follower, poseOnField);
        hiveAim.start();
        lights = new IndicatorLights(hardwareMap);
    }

    /**
     * Updates hiveAim (see HiveAim.update()) and shows on the lights whether it has tags in view.
     * Call once per loop, in a loop that also calls follower.update().
     */
    public void updateHiveAim(boolean redAlliance, boolean aimHeld) {
        hiveAim.update(redAlliance, aimHeld);
        lights.showTagsInView(hiveAim.hasTarget());
    }

    /** Stops the Limelight and turns the lights off; call as the OpMode ends. */
    public void stop() {
        hiveAim.stop();
        lights.off();
    }

    /**
     * Adds a line for each device saying whether it's working, for the init screen, with captions
     * from Utils.column(). The Pinpoint's status is as of the last follower.update().
     */
    public void addInitTelemetry(Telemetry telemetry) {
        telemetry.addData(Utils.column("Pinpoint"), followerTelemetry.pinpointStatus());
        telemetry.addData(Utils.column("Shooter"), shooter.isPresent()
                ? "found" : "not found -- shooter won't run");
        telemetry.addData(Utils.column("Gates"), gates.describeStatus());
        telemetry.addData(Utils.column("Intake"), intake.isPresent()
                ? "found" : "not found -- intake won't run");
        telemetry.addData(Utils.column("Limelight"), hiveAim.hasCamera()
                ? "found" : "not found -- no camera aiming or CELL-ready wait");
        telemetry.addData(Utils.column("Lights"), lights.describeFound());
    }
}
