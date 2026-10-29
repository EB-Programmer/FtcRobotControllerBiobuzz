package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The intake motor. It's optional hardware: if no motor named "intake" is configured, every
 * method quietly does nothing and isPresent() is false, so OpModes still run without it.
 */
public class Intake {
    public static final String HARDWARE_NAME = "intake";
    public static final double DEFAULT_POWER = 0.8;

    public enum State { OFF, FORWARD, REVERSE }

    private final DcMotor motor;
    private State state = State.OFF;
    private double power = DEFAULT_POWER;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.tryGet(DcMotor.class, HARDWARE_NAME);
        if (motor != null) {
            // Intake.State.FORWARD should pull game pieces in. If it pushes them out instead,
            // change this to FORWARD.
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public boolean isPresent() {
        return motor != null;
    }

    public State getState() {
        return state;
    }

    public double getPower() {
        return power;
    }

    /** Sets how hard FORWARD and REVERSE run (0 to 1), taking effect right away if running. */
    public void setPower(double newPower) {
        if (newPower != power) {
            power = newPower;
            set(state);
        }
    }

    public void set(State newState) {
        state = newState;
        if (motor == null) {
            return;
        }
        switch (newState) {
            case FORWARD: motor.setPower(power); break;
            case REVERSE: motor.setPower(-power); break;
            default: motor.setPower(0); break;
        }
    }
}
