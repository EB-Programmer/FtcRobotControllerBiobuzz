package org.firstinspires.ftc.teamcode.routes;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.HiveAim;
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
    private final ElapsedTime autonTimer;
    private final Intake intake;
    private final HiveAim hiveAim;
    private final List<String> notes = new ArrayList<>();

    /**
     * autonTimer is the auton's clock, started when the route starts; hiveAim must be updated by
     * the hold loop, so it reflects what the camera sees while actions wait.
     */
    public RouteActions(BooleanSupplier isActive, HoldLoop holdLoop, ElapsedTime autonTimer,
                        Intake intake, HiveAim hiveAim) {
        this.isActive = isActive;
        this.holdLoop = holdLoop;
        this.autonTimer = autonTimer;
        this.intake = intake;
        this.hiveAim = hiveAim;
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
     * Holds position until the up CELL is ready to shoot at: our alliance's upright HIVE tags in
     * view and not moving for a moment, so a CELL still swinging up after a HIVE TIP isn't shot
     * at (see HiveAim.isCellReady()). A CELL that's already up and still takes about half a
     * second. Gives up once the auton clock reaches untilAutonSeconds -- a time in the 30 s auton,
     * not a length of wait -- so there's still time left for the rest of the route, and returns
     * right away if there's no Limelight. Returns whether the CELL got ready; giving up, or
     * having no Limelight, is noted in the end-of-run summary.
     */
    public boolean waitForCellReady(double untilAutonSeconds) {
        if (!hiveAim.hasCamera()) {
            notes.add("skipped wait for CELL ready: no Limelight");
            return false;
        }
        long millisLeft = Math.round(untilAutonSeconds * 1000 - autonTimer.milliseconds());
        return runUntil(
                String.format(Locale.US, "wait for CELL ready (until %.1fs)", untilAutonSeconds),
                hiveAim::isCellReady, Math.max(0, millisLeft));
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
