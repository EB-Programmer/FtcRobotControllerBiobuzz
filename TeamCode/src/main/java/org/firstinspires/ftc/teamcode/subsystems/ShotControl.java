package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Runs the shooter and gate 1 together, the same way in teleop and auton. While shooting, the
 * shooter runs at the requested speed and the gate stays closed until the shooter passes
 * READY_RPM or SPIN_UP_TIMEOUT_SECONDS have passed since shooting started, whichever comes
 * first; then the gate opens and stays open until shooting stops. While not shooting, the
 * shooter coasts and the gate is closed.
 */
public class ShotControl {
    public static final double READY_RPM = 3200;
    // Opens the gate anyway after this long, so a shooter that never gets up to speed (or a
    // missing encoder reading) can't keep it shut
    public static final double SPIN_UP_TIMEOUT_SECONDS = 5;

    private final Shooter shooter;
    private final ShooterGates.Gate gate;
    private final ElapsedTime shootingTimer = new ElapsedTime();
    private boolean shooting = false;

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
        }
        shooter.setRpm(rpm);
        if (shooter.getRpm() > READY_RPM || shootingTimer.seconds() >= SPIN_UP_TIMEOUT_SECONDS) {
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

    /** Seconds since shooting started, or 0 while not shooting. */
    public double secondsShooting() {
        return shooting ? shootingTimer.seconds() : 0;
    }
}
