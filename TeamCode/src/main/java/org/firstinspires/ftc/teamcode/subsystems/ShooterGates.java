package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The two servos that gate balls into the shooter. Each gate is always either closed, which holds
 * balls back and is where both start, or open, which lets them through. They're optional
 * hardware: either one, or both, can be left unconfigured, and the rest still work.
 */
public class ShooterGates {
    // Servo positions, 0 to 1. Each gate has its own pair, since two servos mounted as mirror
    // images usually need different positions. NaN means not measured yet: a gate doesn't move
    // its servo toward a NaN position, so an unmeasured value can't drive a gate into the frame.
    private static final double GATE1_CLOSED = Double.NaN;
    private static final double GATE1_OPEN = Double.NaN;
    private static final double GATE2_CLOSED = Double.NaN;
    private static final double GATE2_OPEN = Double.NaN;

    public final Gate gate1;
    public final Gate gate2;

    /** Finds the servos and closes both gates, so they're closed from init on. */
    public ShooterGates(HardwareMap hardwareMap) {
        gate1 = new Gate(hardwareMap, "shooterGate1", GATE1_CLOSED, GATE1_OPEN);
        gate2 = new Gate(hardwareMap, "shooterGate2", GATE2_CLOSED, GATE2_OPEN);
    }

    public void closeAll() {
        gate1.close();
        gate2.close();
    }

    /** Which gates were found and whether they can move, for the init screen. */
    public String describeStatus() {
        int found = (gate1.isPresent() ? 1 : 0) + (gate2.isPresent() ? 1 : 0);
        if (found == 0) {
            return "not found -- gates won't move";
        }
        String status = found == 2 ? "found" : "1 of 2 found";
        boolean unmeasured = (gate1.isPresent() && !gate1.hasPositions())
                || (gate2.isPresent() && !gate2.hasPositions());
        return unmeasured ? status + " -- positions not set, won't move" : status;
    }

    /** One gate's servo. Without the servo configured, every method quietly does nothing. */
    public static class Gate {
        private final Servo servo;
        private double closedPosition;
        private double openPosition;
        private boolean open = false;

        private Gate(HardwareMap hardwareMap, String hardwareName,
                     double closedPosition, double openPosition) {
            servo = hardwareMap.tryGet(Servo.class, hardwareName);
            this.closedPosition = closedPosition;
            this.openPosition = openPosition;
            moveServo();
        }

        public boolean isPresent() {
            return servo != null;
        }

        /** Whether both positions are set, so the gate can move both ways. */
        public boolean hasPositions() {
            return !Double.isNaN(closedPosition) && !Double.isNaN(openPosition);
        }

        public boolean isOpen() {
            return open;
        }

        /** The position for the state the gate is in, open or closed; NaN if not set. */
        public double getPosition() {
            return open ? openPosition : closedPosition;
        }

        /**
         * Changes the position for the state the gate is in, open or closed, and moves the servo
         * there, for finding the values to put in the constants. The change lasts until the
         * OpMode ends.
         */
        public void setPosition(double position) {
            if (open) {
                openPosition = position;
            } else {
                closedPosition = position;
            }
            moveServo();
        }

        public void open() {
            setOpen(true);
        }

        public void close() {
            setOpen(false);
        }

        /** Opens or closes the gate. Safe to call every loop; it only moves on a change. */
        public void setOpen(boolean newOpen) {
            if (newOpen == open) {
                return;
            }
            open = newOpen;
            moveServo();
        }

        private void moveServo() {
            double position = getPosition();
            if (servo != null && !Double.isNaN(position)) {
                servo.setPosition(position);
            }
        }
    }
}
