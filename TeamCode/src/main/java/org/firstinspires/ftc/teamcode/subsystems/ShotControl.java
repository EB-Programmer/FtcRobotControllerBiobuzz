package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * Runs the shooter and gate 1 together, the same way in teleop and auton. While shooting, the
 * shooter runs at the requested speed and the gate stays closed until the shooter reaches
 * READY_FRACTION of that speed or SPIN_UP_TIMEOUT_SECONDS have passed since shooting started,
 * whichever comes first; then the gate opens and stays open until shooting stops. While not shooting, the
 * shooter coasts and the gate is closed.
 */
public class ShotControl {
    // How close to the requested speed counts as up to speed, as a fraction of it. Just under 1,
    // since the speed wavers a little around its target and might never quite reach it.
    public static final double READY_FRACTION = 0.95;
    // Opens the gate anyway after this long, so a shooter that never gets up to speed (or a
    // missing encoder reading) can't keep it shut
    public static final double SPIN_UP_TIMEOUT_SECONDS = 5;

    private final Shooter shooter;
    private final ShooterGates.Gate gate;
    private final ElapsedTime shootingTimer = new ElapsedTime();
    private boolean shooting = false;
    // For telemetry while tuning: the speed the latest shot started from (NaN before the first),
    // the fastest it got, and how long it took to get up to speed (NaN if it hasn't yet)
    private double startRpm = Double.NaN;
    private double peakRpm = 0;
    private double spinUpSeconds = Double.NaN;

    public ShotControl(Shooter shooter, ShooterGates.Gate gate) {
        this.shooter = shooter;
        this.gate = gate;
    }

    /**
     * Shoots while shoot is true, at rpm, and stops when it's false; call once per loop. Shooting
     * starts its spin-up clock on the first call with shoot true.
     */
    public void update(boolean shoot, double rpm) {
        if (!shoot) {
            stop();
            return;
        }
        if (!shooting) {
            shooting = true;
            shootingTimer.reset();
            startRpm = shooter.getRpm();
            peakRpm = startRpm;
            spinUpSeconds = Double.NaN;
        }
        shooter.setRpm(rpm);
        double currentRpm = shooter.getRpm();
        peakRpm = Math.max(peakRpm, currentRpm);
        // A speed of 0 or less leaves the shooter stopped, which doesn't count as up to speed
        boolean upToSpeed = rpm > 0 && currentRpm >= READY_FRACTION * rpm;
        if (upToSpeed && Double.isNaN(spinUpSeconds)) {
            spinUpSeconds = shootingTimer.seconds();
        }
        if (upToSpeed || shootingTimer.seconds() >= SPIN_UP_TIMEOUT_SECONDS) {
            gate.open();
        }
    }

    /**
     * Runs the shooter at rpm with the gate closed, so a shot that follows starts already up to
     * speed. It keeps running, with no further calls, until update() or stop(). Not shooting:
     * the spin-up clock starts at the next update(shoot = true).
     */
    public void warmUp(double rpm) {
        shooting = false;
        shooter.setRpm(rpm);
        gate.close();
    }

    /** Coasts the shooter down and closes the gate. */
    public void stop() {
        shooting = false;
        shooter.setRpm(0);
        gate.close();
    }

    public boolean isGateOpen() {
        return gate.isOpen();
    }

    /**
     * How the latest shot's spin-up went, for tuning the shooter's speed control, e.g.
     * "1.42 s from 0 RPM, peak 3080 RPM".
     */
    public String describeSpinUp() {
        if (Double.isNaN(startRpm)) {
            return "no shot yet -- hold the trigger to measure";
        }
        String time;
        if (!Double.isNaN(spinUpSeconds)) {
            time = String.format(Locale.US, "%.2f s", spinUpSeconds);
        } else if (shooting) {
            time = String.format(Locale.US, "%.1f s so far", shootingTimer.seconds());
        } else {
            time = "never got up to speed";
        }
        return String.format(Locale.US, "%s from %.0f RPM, peak %.0f RPM",
                time, startRpm, peakRpm);
    }

    /** Seconds since shooting started, or 0 while not shooting. */
    public double secondsShooting() {
        return shooting ? shootingTimer.seconds() : 0;
    }
}
