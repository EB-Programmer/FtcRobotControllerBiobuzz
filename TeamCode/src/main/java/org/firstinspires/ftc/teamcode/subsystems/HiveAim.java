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
 * other CELL's cluster is seen upside down and ignored (see TagCluster and chooseAimCluster()).
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
    private final List<TagCluster> clusters = new ArrayList<>();
    private TagCluster aimCluster = null;
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
     * Re-reads the Limelight's latest result, picks out the up CELL's cluster from our alliance's
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
            clusters.clear();
            aimCluster = null;
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

        clusters.clear();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (isAllianceTag(tag.getFiducialId(), redAlliance)) {
                clusterFor(tag.getFiducialId()).add(tag, LimelightMount.tagFieldPosition(
                        tag.getTargetPoseCameraSpace(), robotAtCapture));
            }
        }
        aimCluster = chooseAimCluster();
        tx = aimCluster == null ? 0 : aimCluster.middleTx();

        if (aimCluster != null) {
            Pose middle = aimCluster.middleFieldPosition();
            boolean turningSlowly = Math.abs(Math.toDegrees(follower.getAngularVelocity()))
                    < MAX_MEMORY_UPDATE_TURN_RATE_DEG;
            if (middle != null && turningSlowly) {
                memory.remember(middle);
            }
        }
    }

    public boolean hasTarget() {
        return aimCluster != null;
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
            return String.format(Locale.US, "%s, cell %s, %d tags, tx %.1f°%s",
                    state, aimCluster.label(), aimCluster.size(), tx, describeIgnored());
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
        Pose hive = memory.getPosition();
        return String.format(Locale.US,
                "%s (seen %.1fs ago at x %.0f y %.0f), heading %.0f° -> %.0f°%s",
                state, memory.secondsSinceSeen(), hive.getX(), hive.getY(),
                headingDeg, targetDeg, describeIgnored());
    }

    /** A line per visible alliance tag, for debugging; see TagDebugTelemetry. */
    public void addTagTelemetry(Telemetry telemetry) {
        TagDebugTelemetry.add(telemetry, clusters, aimCluster);
    }

    /** "; ignoring 34-37 (upside down)" for each visible cluster seen upside down, else "". */
    private String describeIgnored() {
        StringBuilder result = new StringBuilder();
        for (TagCluster cluster : clusters) {
            if (cluster.orientation() == TagCluster.Orientation.DOWN) {
                result.append("; ignoring ").append(cluster.label()).append(" (upside down)");
            }
        }
        return result.toString();
    }

    private TagCluster clusterFor(int tagId) {
        int index = TagCluster.clusterIndex(tagId);
        for (TagCluster cluster : clusters) {
            if (cluster.index() == index) {
                return cluster;
            }
        }
        TagCluster cluster = new TagCluster(index);
        clusters.add(cluster);
        return cluster;
    }

    /**
     * Which visible cluster to aim at, or null for none. A cluster seen upside down is on the
     * CELL that's down, so it's never aimed at. Otherwise a cluster confirmed up by its tag order
     * beats one with too few tags in view to tell, and between equals the nearer one -- whose
     * tags look bigger -- wins, since the down CELL is the far one.
     */
    private TagCluster chooseAimCluster() {
        TagCluster best = null;
        for (TagCluster cluster : clusters) {
            if (cluster.orientation() == TagCluster.Orientation.DOWN) {
                continue;
            }
            if (best == null) {
                best = cluster;
                continue;
            }
            boolean up = cluster.orientation() == TagCluster.Orientation.UP;
            boolean bestUp = best.orientation() == TagCluster.Orientation.UP;
            if ((up && !bestUp) || (up == bestUp && cluster.meanArea() > best.meanArea())) {
                best = cluster;
            }
        }
        return best;
    }

    private static boolean isAllianceTag(int id, boolean redAlliance) {
        return redAlliance
                ? RED_TAG_MIN_ID <= id && id <= RED_TAG_MAX_ID
                : BLUE_TAG_MIN_ID <= id && id <= BLUE_TAG_MAX_ID;
    }
}
