package org.firstinspires.ftc.teamcode.routes;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.BooleanSupplier;

/**
 * What a Route can do between paths. Every helper runs the auton's hold loop while it waits, so
 * the robot keeps holding its position and the telemetry keeps updating, and every helper
 * returns as soon as Stop is pressed.
 */
public class RouteActions {

    /** One pass of the auton's hold loop; activity describes what's happening, for telemetry. */
    public interface HoldLoop {
        void run(String activity);
    }

    private final BooleanSupplier isActive;
    private final HoldLoop holdLoop;
    private final Intake intake;
    private final List<String> notes = new ArrayList<>();

    public RouteActions(BooleanSupplier isActive, HoldLoop holdLoop, Intake intake) {
        this.isActive = isActive;
        this.holdLoop = holdLoop;
        this.intake = intake;
    }

    /**
     * Starts the intake pulling game pieces in. It keeps running, through the paths that follow,
     * until intakeOff() or intakeReverse(); it's turned off when the route ends. Does nothing
     * without an "intake" motor.
     */
    public void intakeOn() {
        intake.set(Intake.State.FORWARD);
    }

    /** Starts the intake pushing game pieces out; it keeps running like intakeOn(). */
    public void intakeReverse() {
        intake.set(Intake.State.REVERSE);
    }

    public void intakeOff() {
        intake.set(Intake.State.OFF);
    }

    public void waitMillis(long milliseconds) {
        String activity = "wait " + milliseconds + " ms";
        ElapsedTime timer = new ElapsedTime();
        while (isActive.getAsBoolean() && timer.milliseconds() < milliseconds) {
            holdLoop.run(activity);
        }
    }

    /**
     * Holds position until done returns true, giving up after timeoutMillis so a condition that
     * never comes true can't stall the auton. Returns whether done came true; a timeout is noted
     * in the end-of-run summary under description.
     */
    public boolean runUntil(String description, BooleanSupplier done, long timeoutMillis) {
        ElapsedTime timer = new ElapsedTime();
        while (isActive.getAsBoolean()) {
            if (done.getAsBoolean()) {
                return true;
            }
            if (timer.milliseconds() >= timeoutMillis) {
                notes.add("timed out: " + description);
                return false;
            }
            holdLoop.run(description);
        }
        return false;
    }

    /**
     * Runs one step's actions, catching anything they throw so a failed action is flagged instead
     * of ending the auton. Returns the step's part of the end-of-run summary -- how long it took,
     * any notes, or the failure -- or "" if it did nothing.
     */
    public String run(Runnable actions) {
        notes.clear();
        ElapsedTime timer = new ElapsedTime();
        String failure = null;
        try {
            actions.run();
        } catch (RuntimeException e) {
            failure = e.toString();
        }
        double seconds = timer.seconds();

        if (failure == null && notes.isEmpty() && seconds < 0.05) {
            return "";
        }
        StringBuilder result = new StringBuilder(String.format(Locale.US, "%.1fs", seconds));
        for (String note : notes) {
            result.append(", ").append(note);
        }
        if (failure != null) {
            result.append(", FAILED: ").append(failure);
        }
        return result.toString();
    }
}
