package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig driveConfig = new MecanumConfig(config -> {
        config.frontLeftName.set("leftFrontDrive");
        config.frontRightName.set("rightFrontDrive");
        config.backLeftName.set("leftRearDrive");
        config.backRightName.set("rightRearDrive");
        config.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        config.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        config.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        config.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(config -> {
        config.name.set("pinpoint");
        config.xPodOffset.set(185.0);
        config.yPodOffset.set(-194.0);
        config.offsetUnits.set(DistanceUnit.MM);
        config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    });

    // headingFeedback, forwardTranslational, strafeTranslational, brake, coast, the brake
    // coefficient matrices, and the achievable-velocity/deceleration fields are all required
    // by ForesightConfig with no defaults -- Foresight throws IllegalStateException as soon as
    // it's constructed until every one of them is set. The Controller.zero values below are an
    // inert placeholder, not a real tune. Run the Foresight Tuner (see Tuning.java) and paste
    // its generated code here before trusting any autonomous that uses this follower.
    public static ForesightConfig followerConfig = new ForesightConfig(config -> {
        config.headingFeedback.set(Controller.zero);
        config.forwardTranslational.set(Controller.zero);
        config.strafeTranslational.set(Controller.zero);
        config.brake.set(Controller.zero);
        config.coast.set(Controller.zero);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, driveConfig),
                new Foresight(followerConfig)
        );
    }
}
