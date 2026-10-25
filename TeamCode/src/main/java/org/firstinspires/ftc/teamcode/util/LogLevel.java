package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Which telemetry lines an OpMode shows: INFO, the default, for normal driving and runs, or
 * DEBUG, which adds lines for working out what the code is doing. On either gamepad, hold X and
 * press d-pad left for INFO or d-pad right for DEBUG. Every run starts at INFO.
 *
 * <pre>
 * logLevel.update(gamepad1, gamepad2);
 * telemetry.addData("Pose", ...);       // always shown
 * if (logLevel.isDebug()) {
 *     telemetry.addData("Motor", ...);  // only shown at DEBUG
 * }
 * logLevel.addTelemetry(telemetry);
 * </pre>
 */
public class LogLevel {
    private boolean debug = false;

    /**
     * Checks both gamepads for the switch combo; call once per loop. It looks at which buttons
     * are held right now rather than at new presses, so holding the combo for several loops just
     * keeps choosing the same level.
     */
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        for (Gamepad gamepad : new Gamepad[]{gamepad1, gamepad2}) {
            if (gamepad.x && gamepad.dpad_left) {
                debug = false;
            } else if (gamepad.x && gamepad.dpad_right) {
                debug = true;
            }
        }
    }

    public boolean isDebug() {
        return debug;
    }

    /** Adds a line showing the current level and how to switch, for the bottom of the screen. */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Log", debug
                ? "DEBUG (X + d-pad left for INFO)"
                : "INFO (X + d-pad right for DEBUG)");
    }
}
