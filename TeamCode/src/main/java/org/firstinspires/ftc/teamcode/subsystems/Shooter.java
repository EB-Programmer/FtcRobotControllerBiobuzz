package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The shooter motor. It's optional hardware: if no motor named "shooter" is configured, every
 * method quietly does nothing and isPresent() is false, so OpModes still run without it.
 */
public class Shooter {
    public static final String HARDWARE_NAME = "shooter";
    private static final double TICKS_PER_REV = 384.5;

    private final DcMotorEx motor;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.tryGet(DcMotorEx.class, HARDWARE_NAME);
        if (motor != null) {
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public boolean isPresent() {
        return motor != null;
    }

    public void setPower(double power) {
        if (motor != null) {
            motor.setPower(power);
        }
    }

    /** Current speed in RPM, or 0 if the shooter isn't present. */
    public double getRpm() {
        return motor == null ? 0 : motor.getVelocity() * 60 / TICKS_PER_REV;
    }
}
