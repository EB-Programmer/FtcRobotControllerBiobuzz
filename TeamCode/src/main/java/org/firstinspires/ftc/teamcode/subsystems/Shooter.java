package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * The shooter motor, run at a target speed in RPM that the hub holds using the motor's encoder,
 * so the speed doesn't sag as the battery drains. It's optional hardware: if no motor named
 * "shooter" is configured, every method quietly does nothing and isPresent() is false, so OpModes
 * still run without it.
 */
public class Shooter {
    public static final String HARDWARE_NAME = "shooter";
    // The goBILDA 5203 Yellow Jacket 6000 RPM motor's free speed at 12 V. Under load, and on a
    // battery below 12 V, it can't quite reach this.
    public static final double MAX_RPM = 6000;
    // Encoder counts per output-shaft revolution of that motor, which has no gearbox (1:1)
    private static final double TICKS_PER_REV = 28;
    // Coefficients for the hub's speed control loop, from the usual starting recipe:
    // F = 32767 / top speed in ticks per second (6000 RPM is 2800 ticks/s), P = F / 10,
    // I = P / 10, D = 0. F does most of the work. For a closer fit, run the shooter at full power,
    // read its actual top speed, and recompute F (and P and I) from that.
    private static final PIDFCoefficients VELOCITY_PIDF =
            new PIDFCoefficients(1.17, 0.117, 0, 11.7);

    private final DcMotorEx motor;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.tryGet(DcMotorEx.class, HARDWARE_NAME);
        if (motor != null) {
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, VELOCITY_PIDF);
            stop();
        }
    }

    public boolean isPresent() {
        return motor != null;
    }

    /**
     * Runs the shooter at rpm, or stops it if rpm is 0 or less. Stopping lets the flywheel coast
     * down: a speed target of 0 would make the hub actively brake it instead.
     */
    public void setRpm(double rpm) {
        if (motor == null) {
            return;
        }
        if (rpm <= 0) {
            stop();
        } else {
            // setVelocity switches the motor into RUN_USING_ENCODER itself
            motor.setVelocity(rpm * TICKS_PER_REV / 60);
        }
    }

    /** Current speed in RPM, or 0 if the shooter isn't present. */
    public double getRpm() {
        return motor == null ? 0 : motor.getVelocity() * 60 / TICKS_PER_REV;
    }

    private void stop() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }
}
