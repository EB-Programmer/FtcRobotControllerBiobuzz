package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The intake motor. It's optional hardware: if no motor named "intake" is configured, every
 * method quietly does nothing and isPresent() is false, so OpModes still run without it.
 */
public class Intake {
    public static final String HARDWARE_NAME = "intake";
    private static final double POWER = 0.9;

    public enum State { OFF, FORWARD, REVERSE }

    private final DcMotor motor;
    private State state = State.OFF;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.tryGet(DcMotor.class, HARDWARE_NAME);
        if (motor != null) {
            // FORWARD should pull game pieces in. If it pushes them out instead, change this to
            // REVERSE.
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public boolean isPresent() {
        return motor != null;
    }

    public State getState() {
        return state;
    }

    public void set(State newState) {
        state = newState;
        if (motor == null) {
            return;
        }
        switch (newState) {
            case FORWARD: motor.setPower(POWER); break;
            case REVERSE: motor.setPower(-POWER); break;
            default: motor.setPower(0); break;
        }
    }
}
