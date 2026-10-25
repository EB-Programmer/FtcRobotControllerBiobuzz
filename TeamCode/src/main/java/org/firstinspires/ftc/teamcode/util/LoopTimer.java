package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Measures how long each pass of a loop takes. */
public class LoopTimer {
    private final ElapsedTime timer = new ElapsedTime();

    public void reset() {
        timer.reset();
    }

    /** Milliseconds since the last lap() or reset(), restarting the count. */
    public double lap() {
        double ms = timer.milliseconds();
        timer.reset();
        return ms;
    }
}
