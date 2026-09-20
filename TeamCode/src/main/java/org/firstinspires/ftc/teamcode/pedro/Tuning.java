package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;

public class Tuning {
    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hw -> new PinpointLocalizer(hw, Constants.localizerConfig),
                hw -> new Mecanum(hw, Constants.driveConfig)
        );
    }
}
