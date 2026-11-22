package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * The two shooter motors, always run together at the same target speed in RPM, which the hub
 * holds for each motor using its encoder, so the speed doesn't sag as the battery drains. They're
 * optional hardware: either one, or both, can be left unconfigured, and the rest still work. With
 * neither, every method quietly does nothing and isPresent() is false, so OpModes still run.
 */
public class Shooter {
    private static final String[] HARDWARE_NAMES = {"shooter1", "shooter2"};
    // The goBILDA 5203 Yellow Jacket 6000 RPM motor's free speed at 12 V. Under load, and on a
    // battery below 12 V, it can't quite reach this.
    public static final double MAX_RPM = 6000;
    // Encoder counts per output-shaft revolution of that motor, which has no gearbox (1:1)
    private static final double TICKS_PER_REV = 28;
    // The hub's power output at full power, in the units its speed control loop works in
    private static final double FULL_POWER_OUTPUT = 32767;
    // Coefficients for the hub's speed control loop, which works in ticks per second and sets
    // power from 0 to FULL_POWER_OUTPUT. F is the power per tick per second of target speed, the
    // power that holds the speed: FULL_POWER_OUTPUT / the top speed at full power (see
    // fullPowerF()). P is the extra power per tick per second the shooter is below its target,
    // which sets how hard it pushes to get there. I slowly makes up any speed still missing.
    // Values tuned in EB Teleop get copied here; don't change this object, copy it.
    public static final PIDFCoefficients VELOCITY_PIDF =
            new PIDFCoefficients(50, 0, 0, 13.0);

    private final List<DcMotorEx> motors = new ArrayList<>();
    // The names of the motors not found, e.g. "shooter2", for the init screen; "" with both found
    private String missing = "";
    // Every motor name in the robot configuration, quoted, for spotting a misnamed shooter motor
    private String configuredMotors = "";

    public Shooter(HardwareMap hardwareMap) {
        for (String name : HARDWARE_NAMES) {
            DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, name);
            if (motor == null) {
                missing += (missing.isEmpty() ? "" : ", ") + name;
                continue;
            }
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motors.add(motor);
        }
        for (String name : hardwareMap.getAllNames(DcMotor.class)) {
            configuredMotors += (configuredMotors.isEmpty() ? "" : ", ") + "\"" + name + "\"";
        }
        setVelocityPidf(VELOCITY_PIDF);
        stop();
    }

    /** Sets every motor's speed control coefficients, for tuning them. */
    public void setVelocityPidf(PIDFCoefficients pidf) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }
    }

    /**
     * The F that matches a top speed of rpm: read rpm with the shooter at full power, which it is
     * with the target set to MAX_RPM.
     */
    public static double fullPowerF(double rpm) {
        return FULL_POWER_OUTPUT / (rpm * TICKS_PER_REV / 60);
    }

    /** Whether at least one shooter motor is configured. */
    public boolean isPresent() {
        return !motors.isEmpty();
    }

    /**
     * Runs the shooter motors at rpm, or stops them if rpm is 0 or less. Stopping lets the
     * flywheels coast down: a speed target of 0 would make the hub actively brake them instead.
     */
    public void setRpm(double rpm) {
        if (rpm <= 0) {
            stop();
            return;
        }
        for (DcMotorEx motor : motors) {
            // setVelocity switches the motor into RUN_USING_ENCODER itself
            motor.setVelocity(rpm * TICKS_PER_REV / 60);
        }
    }

    /**
     * Current speed in RPM of the slower motor, so a shot waiting on the speed waits for both;
     * 0 if no shooter motor is present.
     */
    public double getRpm() {
        double slowest = Double.POSITIVE_INFINITY;
        for (DcMotorEx motor : motors) {
            slowest = Math.min(slowest, rpmOf(motor));
        }
        return motors.isEmpty() ? 0 : slowest;
    }

    /** Each motor's current speed, for telemetry, e.g. "5980 / 6010 RPM". */
    public String describeRpm() {
        StringBuilder result = new StringBuilder();
        for (DcMotorEx motor : motors) {
            if (result.length() > 0) {
                result.append(" / ");
            }
            result.append(String.format(Locale.US, "%.0f", rpmOf(motor)));
        }
        return result.append(" RPM").toString();
    }

    /**
     * Which motors were found, for the init screen. With any missing, it also lists every motor
     * name the robot configuration has, since a missing motor is usually one named differently.
     */
    public String describeFound() {
        if (missing.isEmpty()) {
            return "found";
        }
        String found = motors.isEmpty()
                ? "not found -- shooter won't run"
                : motors.size() + " of " + HARDWARE_NAMES.length + " found -- missing " + missing;
        return found + ". Motors in the configuration: "
                + (configuredMotors.isEmpty() ? "none" : configuredMotors);
    }

    private static double rpmOf(DcMotorEx motor) {
        return motor.getVelocity() * 60 / TICKS_PER_REV;
    }

    private void stop() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
    }
}
