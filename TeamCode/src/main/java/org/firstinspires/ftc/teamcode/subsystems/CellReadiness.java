package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Locale;

/**
 * Whether the up CELL has stopped moving -- after a HIVE TIP, a CELL takes a couple of seconds to
 * swing all the way up -- judged from camera frames taken while the robot holds still. The CELL
 * is ready once its tags have stayed in view, without moving up or down in the picture by more
 * than TY_TOLERANCE_DEG from where they were, for STILL_MILLIS. Any frame with none of its tags
 * in view, or with them moved farther than that, starts the wait over.
 *
 * Up/down position (ty) is used rather than left/right (tx) because the robot wobbles a little
 * in heading while holding its position, which moves the tags left and right in the picture far
 * more than up and down.
 */
final class CellReadiness {
    // On the accepting side, to tighten if the robot shoots before the CELL has stopped: still
    // tags jitter well under this, and it lets through only motion slower than about 2°/s
    private static final double TY_TOLERANCE_DEG = 1.0;
    private static final long STILL_MILLIS = 500;

    private double stillTy;
    private long stillSinceMillis = -1;  // -1 while no tags are in view
    private long lastFrameMillis;

    /** Starts the wait over, for a frame with none of the CELL's tags in view. */
    void clear() {
        stillSinceMillis = -1;
    }

    /** Adds a frame that has tags in view, taken at frameMillis, with them at middleTy. */
    void addFrame(long frameMillis, double middleTy) {
        lastFrameMillis = frameMillis;
        if (stillSinceMillis < 0 || Math.abs(middleTy - stillTy) > TY_TOLERANCE_DEG) {
            stillTy = middleTy;
            stillSinceMillis = frameMillis;
        }
    }

    boolean isReady() {
        return stillSinceMillis >= 0 && lastFrameMillis - stillSinceMillis >= STILL_MILLIS;
    }

    /** E.g. "READY (still 0.8s, ty 12.4°)", "still 0.3s of 0.5s, ty 12.4°", or "no tags". */
    String describe() {
        if (stillSinceMillis < 0) {
            return "no tags";
        }
        double stillSeconds = (lastFrameMillis - stillSinceMillis) / 1000.0;
        return isReady()
                ? String.format(Locale.US, "READY (still %.1fs, ty %.1f°)", stillSeconds, stillTy)
                : String.format(Locale.US, "still %.1fs of %.1fs, ty %.1f°",
                        stillSeconds, STILL_MILLIS / 1000.0, stillTy);
    }
}
