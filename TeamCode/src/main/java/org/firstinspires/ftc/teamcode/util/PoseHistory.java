package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import java.util.ArrayDeque;
import java.util.Iterator;

/**
 * The robot's recent poses with the time each was recorded, so a camera frame -- which shows the
 * field as it was some milliseconds ago -- can be matched to where the robot was when it was
 * captured rather than where it is now.
 */
public class PoseHistory {
    private static final long KEEP_MILLIS = 1000;

    private static final class Entry {
        final long timeMillis;
        final Pose pose;

        Entry(long timeMillis, Pose pose) {
            this.timeMillis = timeMillis;
            this.pose = pose;
        }
    }

    private final ArrayDeque<Entry> entries = new ArrayDeque<>();

    /** Records pose as the robot's pose right now. Call once per loop. */
    public void record(Pose pose) {
        long now = System.currentTimeMillis();
        entries.addLast(new Entry(now, pose));
        while (entries.peekFirst().timeMillis < now - KEEP_MILLIS) {
            entries.removeFirst();
        }
    }

    /**
     * The most recent pose recorded at or before timeMillis (a System.currentTimeMillis() time),
     * or the oldest one kept if timeMillis is earlier than all of them. Null if nothing has been
     * recorded.
     */
    public Pose poseAt(long timeMillis) {
        Iterator<Entry> newestFirst = entries.descendingIterator();
        Entry entry = null;
        while (newestFirst.hasNext()) {
            entry = newestFirst.next();
            if (entry.timeMillis <= timeMillis) {
                break;
            }
        }
        return entry == null ? null : entry.pose;
    }
}
