package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

/**
 * The two RGB indicator lights. Each is configured as a servo, and the position it's set to picks
 * its color. They're optional hardware: either one, or both, can be left unconfigured, and the
 * rest still work.
 */
public class IndicatorLights {
    private static final String[] HARDWARE_NAMES = {"ledIndicator0", "ledIndicator1"};
    private static final double COLOR_OFF = 0.0;
    private static final double COLOR_GREEN = 0.5;

    private final List<Servo> lights = new ArrayList<>();
    private double shownColor = Double.NaN;

    public IndicatorLights(HardwareMap hardwareMap) {
        for (String name : HARDWARE_NAMES) {
            Servo light = hardwareMap.tryGet(Servo.class, name);
            if (light != null) {
                lights.add(light);
            }
        }
        off();
    }

    /** Green while the HIVE tags aiming uses are in view, otherwise off. */
    public void showTagsInView(boolean tagsInView) {
        setColor(tagsInView ? COLOR_GREEN : COLOR_OFF);
    }

    public void off() {
        setColor(COLOR_OFF);
    }

    /** Which lights were found, for the init screen. */
    public String describeFound() {
        if (lights.isEmpty()) {
            return "not found -- no tags-in-view light";
        }
        return lights.size() == HARDWARE_NAMES.length
                ? "found"
                : lights.size() + " of " + HARDWARE_NAMES.length + " found";
    }

    /** Only sends a color when it changes, since this runs every loop. */
    private void setColor(double color) {
        if (color == shownColor) {
            return;
        }
        shownColor = color;
        for (Servo light : lights) {
            light.setPosition(color);
        }
    }
}
