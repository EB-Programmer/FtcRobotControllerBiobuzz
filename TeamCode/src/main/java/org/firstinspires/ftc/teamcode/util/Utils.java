package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Utils {
    private Utils() {
    }

    /**
     * Nudges value up or down by increment on a button press, clamped to
     * [0, maxValue], and reports the resulting value to telemetry under name.
     */
    public static double tuneConstant(Telemetry telemetry, String name, double value,
                                       boolean buttonUp, boolean buttonDown,
                                       double increment, double maxValue) {
        if (buttonUp) {
            value = value + increment;
        } else if (buttonDown) {
            value = value - increment;
        }

        // Snap to a multiple of increment and trim floating-point noise, so repeated presses
        // don't drift (e.g. to 0.30000000000000004)
        value = Math.round(value / increment) * increment;
        value = Math.round(value * 1e6) / 1e6;

        if (value > maxValue) {
            value = maxValue;
        } else if (value < 0) {
            value = 0;
        }

        telemetry.addData(name, value);
        return value;
    }
}
