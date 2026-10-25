package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PoseHistory;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Aims the robot at its alliance's HIVE using the Limelight's view of the AprilTag Clusters on
 * the HIVE CELLS: each alliance has two CELLS with a cluster of four tags each, and which CELL is
 * up changes after each HIVE TIP. The up CELL's cluster is the one to aim at, at its middle; the
 * other CELL's tags are seen upside down (see TagCorners) and left out.
 *
 * It also remembers where on the field the HIVE was last seen (see HiveMemory). With no tags in
 * view, getTurn() turns the robot toward that remembered spot from wherever the robot is now,
 * until tags come back into view and the camera takes over again. Once the robot faces the
 * remembered spot and still can't see any tags (say, something's in the way), aiming hands
 * turning back to the driver -- see isControllingTurn() -- until the aim button is released or
 * tags come into view.
 *
 * The Limelight is optional hardware: without one named "limelight" configured, aiming has
 * nothing to go on and never takes over turning.
 */
public class HiveAim {
    // AprilTag IDs on the HIVE CELLS: red 30-37, blue 38-45
    private static final int RED_TAG_MIN_ID = 30;
    private static final int RED_TAG_MAX_ID = 37;
    private static final int BLUE_TAG_MIN_ID = 38;
    private static final int BLUE_TAG_MAX_ID = 45;
    private static final int APRILTAG_PIPELINE = 4;  // Limelight pipeline slot
    private static final double KP = 0.02;  // turn power per degree off target
    private static final double MAX_TURN = 0.5;
    // Least turn power used outside tolerance, so the robot doesn't stall just short of its
    // target when KP alone asks for too little power to move. Too high makes it overshoot back
    // and forth; tune on the robot.
    private static final double MIN_TURN = 0.06;
    private static final double TOLERANCE_DEG = 1.0;
    private static final long MAX_RESULT_STALENESS = 100;  // milliseconds
    // Frames taken while spinning faster than this are blurry and timing-sensitive, so they
    // don't update the memory
    private static final double MAX_MEMORY_UPDATE_TURN_RATE_DEG = 90.0;  // degrees per second

    private final Limelight3A limelight;
    private final Follower follower;
    private final PoseHistory poseHistory = new PoseHistory();
    private final HiveMemory memory = new HiveMemory();
    // This frame's alliance tags: upright ones, all on the up CELL, to aim at; and upside-down
    // ones, on the down CELL, left out
    private final TagCluster aimTags = new TagCluster();
    private final List<LLResultTypes.FiducialResult> upsideDownTags = new ArrayList<>();
    private boolean cornersMissing = false;
    private double tx = 0;
    private long lastResultMillis = -1;

    private Boolean memoryRedAlliance = null;
    private boolean aimHeld = false;
    private boolean reachedRememberedHeading = false;

    public HiveAim(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.setPollRateHz(100);  // how many times/sec to request data
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
        }
    }

    /** Starts the Limelight sending results; call before update(). */
    public void start() {
        if (limelight != null) {
            limelight.start();
        }
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    /**
     * Re-reads the Limelight's latest result, picks out the up CELL's tags from our alliance's
     * tags, and updates the remembered HIVE position from it. aimHeld is whether the driver is
     * holding the aim button. Call once per loop, in a loop that also calls follower.update().
     */
    public void update(boolean redAlliance, boolean aimHeld) {
        this.aimHeld = aimHeld;
        poseHistory.record(follower.getPose());
        if (memoryRedAlliance == null || memoryRedAlliance != redAlliance) {
            memoryRedAlliance = redAlliance;
            memory.clear();
            lastResultMillis = -1;  // so the current frame is re-read for the new alliance's tags
        }

        readCamera(redAlliance);

        // Latched, so the driver turning off the remembered heading doesn't make aiming take
        // turning back and snap the robot back onto it
        if (!aimHeld || hasTarget()) {
            reachedRememberedHeading = false;
        } else if (!memory.isEmpty()
                && Math.abs(memory.headingErrorDeg(follower.getPose())) < TOLERANCE_DEG) {
            reachedRememberedHeading = true;
        }
    }

    private void readCamera(boolean redAlliance) {
        LLResult result = limelight == null ? null : limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() >= MAX_RESULT_STALENESS) {
            aimTags.clear();
            upsideDownTags.clear();
            cornersMissing = false;
            tx = 0;
            return;
        }
        // The same result comes back until the Limelight sends a new one, which can take several
        // loops; handling each only once keeps a frame from counting more than once in memory
        if (result.getControlHubTimeStamp() == lastResultMillis) {
            return;
        }
        lastResultMillis = result.getControlHubTimeStamp();

        long captureMillis = result.getControlHubTimeStamp()
                - Math.round(result.getCaptureLatency() + result.getTargetingLatency());
        Pose robotAtCapture = poseHistory.poseAt(captureMillis);

        // Only one of our alliance's two CELLS is up at a time, so once the upside-down tags --
        // on the CELL that's down -- are set aside, every tag left is on the up CELL
        aimTags.clear();
        upsideDownTags.clear();
        cornersMissing = false;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (!isAllianceTag(tag.getFiducialId(), redAlliance)) {
                continue;
            }
            if (!TagCorners.hasCorners(tag)) {
                cornersMissing = true;
            }
            if (TagCorners.isUpsideDown(tag)) {
                upsideDownTags.add(tag);
            } else {
                aimTags.add(tag, LimelightMount.tagFieldPosition(
                        tag.getTargetPoseCameraSpace(), robotAtCapture));
            }
        }
        tx = aimTags.isEmpty() ? 0 : aimTags.middleTx();

        if (!aimTags.isEmpty()) {
            Pose middle = aimTags.middleFieldPosition();
            boolean turningSlowly = Math.abs(Math.toDegrees(follower.getAngularVelocity()))
                    < MAX_MEMORY_UPDATE_TURN_RATE_DEG;
            if (middle != null && turningSlowly) {
                memory.remember(middle);
            }
        }
    }

    public boolean hasTarget() {
        return !aimTags.isEmpty();
    }

    /**
     * Whether aiming should replace the driver's turning: the aim button is held, there's
     * something to aim at (tags in view, or a remembered HIVE position), and the robot hasn't
     * already reached the remembered heading without seeing any tags there.
     */
    public boolean isControllingTurn() {
        return aimHeld && (hasTarget() || !memory.isEmpty()) && !reachedRememberedHeading;
    }

    /**
     * Turn power that points the robot at the HIVE, in Pedro Pathing's convention (positive turns
     * counterclockwise): from the camera when tags are in view, otherwise toward the remembered
     * HIVE position. 0 with nothing to aim at or once within tolerance.
     */
    public double getTurn() {
        // Positive error means the HIVE is counterclockwise of where the robot is facing
        double errorDeg;
        if (hasTarget()) {
            errorDeg = -tx;  // positive tx means the tags are to the right (clockwise)
        } else if (!memory.isEmpty()) {
            errorDeg = memory.headingErrorDeg(follower.getPose());
        } else {
            return 0;
        }
        if (Math.abs(errorDeg) < TOLERANCE_DEG) {
            return 0;
        }
        double power = Math.max(MIN_TURN, Math.min(MAX_TURN, KP * Math.abs(errorDeg)));
        return Math.signum(errorDeg) * power;
    }

    /** What aiming is doing, or would do if the aim button were held, for telemetry. */
    public String describeStatus() {
        if (limelight == null) {
            return "no camera -- no \"limelight\" configured";
        }
        if (hasTarget()) {
            String state = Math.abs(tx) < TOLERANCE_DEG ? "LOCKED (camera)"
                    : aimHeld ? "TURNING to tags (camera)"
                    : "tags in view (camera)";
            return String.format(Locale.US, "%s, %d tags, tx %.1f°%s",
                    state, aimTags.size(), tx, describeIgnored());
        }
        if (memory.isEmpty()) {
            return "no data" + describeIgnored();
        }
        String state = reachedRememberedHeading
                ? "ON REMEMBERED HEADING -- no tags in view, gamepad 1 turns"
                : aimHeld ? "TURNING to remembered HIVE"
                : "remembered HIVE";
        Pose robot = follower.getPose();
        double headingDeg = Utils.degrees(robot.getHeading());
        double targetDeg = Utils.degrees(
                robot.getHeading() + Math.toRadians(memory.headingErrorDeg(robot)));
        return String.format(Locale.US, "%s, heading %.0f° -> %.0f°%s",
                state, headingDeg, targetDeg, describeIgnored());
    }

    /**
     * Where the remembered HIVE is, for telemetry: how far away and which way to turn to face it
     * from where the robot is now, and how long ago tags last updated it.
     */
    public String describeHive() {
        if (limelight == null) {
            return "n/a -- no camera";
        }
        if (memory.isEmpty()) {
            return "not seen yet (needs tags in view, with Full 3D on in the Limelight)";
        }
        Pose robot = follower.getPose();
        Pose hive = memory.getPosition();
        double turnDeg = memory.headingErrorDeg(robot);  // positive is counterclockwise (left)
        long inches = Math.round(robot.distanceFrom(hive));
        return String.format(Locale.US, "%d'%d\" away, %.0f° %s, seen %.1fs ago",
                inches / 12, inches % 12, Math.abs(turnDeg), turnDeg >= 0 ? "left" : "right",
                memory.secondsSinceSeen());
    }

    /** A line per visible alliance tag, for debugging; see TagDebugTelemetry. */
    public void addTagTelemetry(Telemetry telemetry) {
        TagDebugTelemetry.add(telemetry, aimTags, upsideDownTags);
    }

    /**
     * Notes for the end of the status line: which tags were left out for being upside down, e.g.
     * "; ignoring upside-down 30, 33", and a warning when tags arrive without corners, so
     * upside-down ones can't be spotted. "" when there's nothing to note.
     */
    private String describeIgnored() {
        StringBuilder result = new StringBuilder();
        if (!upsideDownTags.isEmpty()) {
            result.append("; ignoring upside-down");
            for (int i = 0; i < upsideDownTags.size(); i++) {
                result.append(i == 0 ? " " : ", ").append(upsideDownTags.get(i).getFiducialId());
            }
        }
        if (cornersMissing) {
            result.append("; NO CORNERS from Limelight -- can't spot upside-down tags");
        }
        return result.toString();
    }

    private static boolean isAllianceTag(int id, boolean redAlliance) {
        return redAlliance
                ? RED_TAG_MIN_ID <= id && id <= RED_TAG_MAX_ID
                : BLUE_TAG_MIN_ID <= id && id <= BLUE_TAG_MAX_ID;
    }
}
