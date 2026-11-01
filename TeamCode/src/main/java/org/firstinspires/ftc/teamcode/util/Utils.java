package org.firstinspires.ftc.teamcode.util;

import java.util.Locale;

public final class Utils {
    private static final int COLUMN_WIDTH = "Limelight".length();  // the longest caption in use

    private Utils() {
    }

    /**
     * Nudges value up or down by increment on a button press, clamped to [0, maxValue], and
     * returns the result.
     */
    public static double tuneConstant(double value, boolean buttonUp, boolean buttonDown,
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

        return value;
    }

    /**
     * Pads a telemetry caption so the values after it line up in a column, which takes
     * telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE) to show properly.
     */
    public static String column(String caption) {
        return String.format(Locale.US, "%-" + COLUMN_WIDTH + "s", caption);
    }

    /** An angle in radians as degrees, normalized to [0, 360). */
    public static double degrees(double radians) {
        double degrees = Math.toDegrees(radians) % 360;
        return degrees < 0 ? degrees + 360 : degrees;
    }
}
