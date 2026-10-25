package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * A settings-style telemetry menu for a LinearOpMode's init loop. Each choice is one row showing
 * its current value; on either gamepad, d-pad up/down picks a row and d-pad left/right changes
 * that row's value. Every choice starts on its first option, so there's always a complete set of
 * values to run with, and what's on screen is exactly what's used when Start is pressed.
 *
 * <pre>
 * InitMenu menu = new InitMenu("Auton setup");
 * InitMenu.Choice&lt;String&gt; side = menu.choice("Side", "Left", "Right");
 * InitMenu.Choice&lt;Integer&gt; delay = menu.choice("Delay (s)", 0, 2, 5);
 * while (opModeInInit()) {
 *     menu.update(gamepad1, gamepad2, telemetry);
 *     telemetry.update();
 * }
 * // side.get() and delay.get() hold the driver's choices
 * </pre>
 */
public class InitMenu {

    /** One row of the menu; get() returns its current value. */
    public static final class Choice<T> {
        private final String name;
        private final T[] options;
        private final Function<T, String> label;
        private int index = 0;
        private boolean changed = true;

        private Choice(String name, T[] options, Function<T, String> label) {
            this.name = name;
            this.options = options;
            this.label = label;
        }

        public T get() {
            return options[index];
        }

        /**
         * True the first time it's called and afterward whenever the value has changed since the
         * previous call -- a cue to redo any work that depends on the value.
         */
        public boolean changed() {
            boolean result = changed;
            changed = false;
            return result;
        }

        private void step(int delta) {
            index = Math.floorMod(index + delta, options.length);
            changed = true;
        }
    }

    private final String title;
    private final List<Choice<?>> choices = new ArrayList<>();
    private int row = 0;

    public InitMenu(String title) {
        this.title = title;
    }

    /** Adds a row whose options are shown using label. The first option is the default. */
    public <T> Choice<T> choice(String name, T[] options, Function<T, String> label) {
        if (options.length == 0) {
            throw new IllegalArgumentException("InitMenu choice \"" + name + "\" has no options");
        }
        Choice<T> choice = new Choice<>(name, options.clone(), label);
        choices.add(choice);
        return choice;
    }

    /** Adds a row whose options are shown as-is. The first option is the default. */
    @SafeVarargs
    public final <T> Choice<T> choice(String name, T... options) {
        return choice(name, options, String::valueOf);
    }

    /**
     * Handles one init-loop pass of gamepad input and adds the menu to telemetry. The caller
     * still calls telemetry.update().
     */
    public void update(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        boolean up = gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed();
        boolean down = gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed();
        boolean left = gamepad1.dpadLeftWasPressed() | gamepad2.dpadLeftWasPressed();
        boolean right = gamepad1.dpadRightWasPressed() | gamepad2.dpadRightWasPressed();

        if (!choices.isEmpty()) {
            if (up) {
                row = Math.floorMod(row - 1, choices.size());
            } else if (down) {
                row = Math.floorMod(row + 1, choices.size());
            } else if (left) {
                choices.get(row).step(-1);
            } else if (right) {
                choices.get(row).step(1);
            }
        }

        telemetry.addLine(title);
        telemetry.addLine("D-pad up/down: pick row. Left/right: change value.");
        if (gamepad1.getGamepadId() == Gamepad.ID_UNASSOCIATED
                && gamepad2.getGamepadId() == Gamepad.ID_UNASSOCIATED) {
            telemetry.addLine("No gamepad connected -- press Start+A (or Start+B) to connect one.");
        }
        telemetry.addLine("------------");
        for (int i = 0; i < choices.size(); i++) {
            telemetry.addLine(formatRow(choices.get(i), i == row));
        }
        telemetry.addLine("------------");
    }

    private static <T> String formatRow(Choice<T> choice, boolean highlighted) {
        String value = choice.label.apply(choice.get());
        return highlighted
                ? "> " + choice.name + ": < " + value + " >"
                : "   " + choice.name + ": " + value;
    }
}
