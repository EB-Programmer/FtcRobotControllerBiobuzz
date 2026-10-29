package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.util.PoseHistory;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Aims the robot at its alliance's HIVE using the Limelight's view of the AprilTag Clusters on
 * the HIVE CELLS: each alliance has two CELLS with a cluster of four tags each, and which CELL is
 * up changes after each HIVE TIP. The up CELL's cluster is the one to aim at, at its middle; the
 * other CELL's tags are seen either upside down (see TagCorners) or lying nearly flat (see
 * TagTilt), and left out.
 *
 * It also remembers where on the field the HIVE was last seen (see HiveMemory). With no tags in
 * view, getTurn() turns the robot toward that remembered spot from wherever the robot is now,
 * until tags come back into view and the camera takes over again. Before any tags have been
 * seen, it turns toward the nearer of our alliance's two CELLS on the field map instead, as long
 * as the robot's pose is in field coordinates (it is after EB Auton). Once the robot faces
 * either spot and still can't see any tags (say, something's in the way), aiming hands turning
 * back to the driver -- see isControllingTurn() -- until the aim button is released or tags come
 * into view.
 *
 * Either way, the turning itself steers by the Pinpoint's heading, which is up to date every
 * loop: the camera only decides which heading faces the tags. Each camera frame is some
 * milliseconds old by the time it arrives, and steering straight off it would keep the robot
 * turning after it got there, swinging back and forth past the target.
 *
 * It also tracks whether the up CELL has stopped moving, for an auton waiting on a HIVE TIP --
 * see CellReadiness and isCellReady().
 *
 * The Limelight is optional hardware: without one named "limelight" configured, aiming only has
 * the field map to go on.
 */
public class HiveAim {
    // AprilTag IDs on the HIVE CELLS: red 30-37, blue 38-45
    private static final int RED_TAG_MIN_ID = 30;
    private static final int RED_TAG_MAX_ID = 37;
    private static final int BLUE_TAG_MIN_ID = 38;
    private static final int BLUE_TAG_MAX_ID = 45;
    private static final int APRILTAG_PIPELINE = 4;  // Limelight pipeline slot
    private static final double KP = 0.02;  // turn power per degree off target
    // Turn power pushing against the turning, per degree per second the robot is turning, so it
    // slows down on the way in rather than swinging past. Raise it if the robot still overshoots;
    // lower it if it creeps in too slowly at the end.
    private static final double KD = 0.003;
    private static final double MAX_TURN = 0.5;
    // Least turn power used when the robot has stopped short of its target, so it doesn't stall
    // there when KP alone asks for too little power to move. Too high makes it overshoot back
    // and forth; tune on the robot.
    private static final double MIN_TURN = 0.06;
    private static final double TOLERANCE_DEG = 1.0;
    // Turning slower than this counts as stopped
    private static final double STOPPED_TURN_RATE_DEG = 5.0;  // degrees per second
    private static final long MAX_RESULT_STALENESS = 100;  // milliseconds
    // Frames taken while spinning faster than this are blurry and timing-sensitive, so they
    // don't update the memory
    private static final double MAX_MEMORY_UPDATE_TURN_RATE_DEG = 90.0;  // degrees per second
    // Where our alliance's two CELLS are on the field, for aiming before any tags have been
    // seen. In the red alliance's coordinates, which every OpMode uses whichever alliance we're
    // on: the blue side of the field is the red side turned halfway around.
    private static final Pose[] MAP_CELL_POSITIONS = {new Pose(58, 55.25), new Pose(58, 86.25)};

    private final Limelight3A limelight;
    private final Follower follower;
    private final GoBildaPinpointDriver pinpoint;
    private final boolean poseOnField;
    private final PoseHistory poseHistory = new PoseHistory();
    private final HiveMemory memory = new HiveMemory();
    // This frame's alliance tags: upright, standing ones, all on the up CELL, to aim at; and
    // upside-down or flat ones, on the down CELL, left out
    private final TagCluster aimTags = new TagCluster();
    private final List<LLResultTypes.FiducialResult> upsideDownTags = new ArrayList<>();
    private final List<LLResultTypes.FiducialResult> flatTags = new ArrayList<>();
    private final CellReadiness cellReadiness = new CellReadiness();
    private boolean cornersMissing = false;
    private boolean tiltMissing = false;
    // The heading, in radians, that faces the middle of the tags in view, from the latest frame;
    // only meaningful while hasTarget()
    private double cameraTargetHeading = 0;
    private long lastResultMillis = -1;

    private Boolean memoryRedAlliance = null;
    private boolean aimHeld = false;
    private boolean reachedExpectedHeading = false;

    /**
     * poseOnField is whether the follower's pose is in field coordinates, so the field map can be
     * used to aim before any tags have been seen.
     */
    public HiveAim(HardwareMap hardwareMap, Follower follower, boolean poseOnField) {
        this.follower = follower;
        this.poseOnField = poseOnField;
        Localizer localizer = follower.getPoseTracker().getLocalizer();
        pinpoint = localizer instanceof PinpointLocalizer
                ? ((PinpointLocalizer) localizer).getPinpoint()
                : null;
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
            cellReadiness.clear();
            lastResultMillis = -1;  // so the current frame is re-read for the new alliance's tags
        }

        readCamera(redAlliance);

        // Latched, so the driver turning off the expected heading doesn't make aiming take
        // turning back and snap the robot back onto it
        Pose expected = expectedTarget();
        if (!aimHeld || hasTarget()) {
            reachedExpectedHeading = false;
        } else if (expected != null
                && Math.abs(headingErrorTo(follower.getPose(), expected)) < TOLERANCE_DEG
                && isStopped()) {
            reachedExpectedHeading = true;
        }
    }

    private void readCamera(boolean redAlliance) {
        LLResult result = limelight == null ? null : limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() >= MAX_RESULT_STALENESS) {
            aimTags.clear();
            upsideDownTags.clear();
            flatTags.clear();
            cornersMissing = false;
            tiltMissing = false;
            cellReadiness.clear();
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

        // Only one of our alliance's two CELLS is up at a time, so once the upside-down and flat
        // tags -- on the CELL that's down -- are set aside, every tag left is on the up CELL
        aimTags.clear();
        upsideDownTags.clear();
        flatTags.clear();
        cornersMissing = false;
        tiltMissing = false;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (!isAllianceTag(tag.getFiducialId(), redAlliance)) {
                continue;
            }
            if (!TagCorners.hasCorners(tag)) {
                cornersMissing = true;
            }
            if (!TagTilt.hasTilt(tag)) {
                tiltMissing = true;
            }
            if (TagCorners.isUpsideDown(tag)) {
                upsideDownTags.add(tag);
            } else if (TagTilt.isLyingFlat(tag)) {
                flatTags.add(tag);
            } else {
                aimTags.add(tag, LimelightMount.tagRobotPosition(tag.getTargetPoseCameraSpace()));
            }
        }
        if (aimTags.isEmpty()) {
            cellReadiness.clear();
            return;
        }
        cellReadiness.addFrame(captureMillis, aimTags.middleTy());

        // The camera sits off to the side of the robot's center (see LimelightMount), so turning
        // until the tags are straight ahead of the camera would leave the robot pointed beside
        // them. With 3D positions, aim the robot's center at them instead; without, the camera's
        // angle is the best there is.
        Pose middle = aimTags.middleRobotPosition();
        double aimErrorDeg = middle != null
                ? Math.toDegrees(Math.atan2(middle.getY(), middle.getX()))
                : -aimTags.middleTx();  // positive tx means the tags are to the right
        // The frame shows the tags as they were when it was captured, so the heading that faces
        // them is worked out from where the robot was facing then
        Pose facing = robotAtCapture != null ? robotAtCapture : follower.getPose();
        cameraTargetHeading = facing.getHeading() + Math.toRadians(aimErrorDeg);

        boolean turningSlowly = Math.abs(turnRateDeg()) < MAX_MEMORY_UPDATE_TURN_RATE_DEG;
        if (middle != null && robotAtCapture != null && turningSlowly) {
            memory.remember(LimelightMount.fieldPosition(middle, robotAtCapture));
        }
    }

    /**
     * Whether the up CELL's tags have been in view and still for a moment (see CellReadiness):
     * the CELL has finished swinging up after a HIVE TIP, or wasn't moving to begin with. Only
     * meaningful while the robot itself holds still.
     */
    public boolean isCellReady() {
        return cellReadiness.isReady();
    }

    /** How close the up CELL is to counting as ready, for telemetry; see isCellReady(). */
    public String describeCellReadiness() {
        return cellReadiness.describe();
    }

    /** Whether a Limelight is configured; without one, hasTarget() is always false. */
    public boolean hasCamera() {
        return limelight != null;
    }

    /**
     * Whether any of our alliance's upright, standing HIVE tags -- the ones aiming uses -- are in
     * view.
     */
    public boolean hasTarget() {
        return !aimTags.isEmpty();
    }

    /**
     * Whether aiming should replace the driver's turning: the aim button is held, there's
     * something to aim at (tags in view, or else see expectedTarget()), and the robot hasn't
     * already reached the expected heading without seeing any tags there.
     */
    public boolean isControllingTurn() {
        return aimHeld && (hasTarget() || expectedTarget() != null) && !reachedExpectedHeading;
    }

    /**
     * Where to aim with no tags in view: the remembered HIVE, or before any tags have been seen,
     * the nearer of our two CELLS on the field map. Null with neither.
     */
    private Pose expectedTarget() {
        return memory.isEmpty() ? nearestMapCell() : memory.getPosition();
    }

    /** The field map's CELL nearest the robot; null unless the pose is in field coordinates. */
    private Pose nearestMapCell() {
        if (!poseOnField) {
            return null;
        }
        Pose robot = follower.getPose();
        Pose nearest = null;
        for (Pose cell : MAP_CELL_POSITIONS) {
            if (nearest == null || robot.distanceFrom(cell) < robot.distanceFrom(nearest)) {
                nearest = cell;
            }
        }
        return nearest;
    }

    /**
     * Turn power that points the robot at the HIVE, in Pedro Pathing's convention (positive turns
     * counterclockwise): toward the tags when they're in view, otherwise toward expectedTarget().
     * 0 with nothing to aim at, or once stopped within tolerance.
     */
    public double getTurn() {
        double errorDeg = headingErrorDeg();
        if (Double.isNaN(errorDeg)) {
            return 0;
        }
        boolean onTarget = Math.abs(errorDeg) < TOLERANCE_DEG;
        if (onTarget && isStopped()) {
            return 0;
        }
        // Pushes toward the target harder the farther off it is, and against the turning harder
        // the faster it's turning -- like a shock absorber
        double power = KP * errorDeg - KD * turnRateDeg();
        if (!onTarget && isStopped()) {
            power = Math.signum(errorDeg) * Math.max(Math.abs(power), MIN_TURN);
        }
        return Math.max(-MAX_TURN, Math.min(MAX_TURN, power));
    }

    /**
     * Degrees to turn from the robot's heading to face what it's aiming at, in (-180, 180],
     * positive counterclockwise (left): the tags in view, otherwise expectedTarget(). NaN with
     * nothing to aim at.
     */
    private double headingErrorDeg() {
        Pose robot = follower.getPose();
        if (hasTarget()) {
            return wrapDeg(Math.toDegrees(cameraTargetHeading - robot.getHeading()));
        }
        Pose expected = expectedTarget();
        return expected == null ? Double.NaN : headingErrorTo(robot, expected);
    }

    /** Signed degrees from robot's heading to face point, in (-180, 180]. */
    private static double headingErrorTo(Pose robot, Pose point) {
        double bearing = Math.atan2(point.getY() - robot.getY(), point.getX() - robot.getX());
        return wrapDeg(Math.toDegrees(bearing - robot.getHeading()));
    }

    /** An angle in degrees, wrapped into (-180, 180]. */
    private static double wrapDeg(double degrees) {
        double radians = Math.toRadians(degrees);
        return Math.toDegrees(Math.atan2(Math.sin(radians), Math.cos(radians)));
    }

    /**
     * How fast the robot is turning, in degrees per second, positive counterclockwise, as of the
     * last follower.update(). Read from the Pinpoint itself: in Pedro Pathing 2.0.2,
     * follower.getAngularVelocity() with a Pinpoint returns the robot's heading instead.
     */
    private double turnRateDeg() {
        return pinpoint == null ? 0 : pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    private boolean isStopped() {
        return Math.abs(turnRateDeg()) < STOPPED_TURN_RATE_DEG;
    }

    /** What aiming is doing, or would do if the aim button were held, for telemetry. */
    public String describeStatus() {
        if (hasTarget()) {
            double aimErrorDeg = headingErrorDeg();
            String state = Math.abs(aimErrorDeg) < TOLERANCE_DEG ? "LOCKED (camera)"
                    : aimHeld ? "TURNING to tags (camera)"
                    : "tags in view (camera)";
            return String.format(Locale.US, "%s, %d tags, %.1f° %s%s",
                    state, aimTags.size(), Math.abs(aimErrorDeg),
                    aimErrorDeg >= 0 ? "left" : "right", describeIgnored());
        }
        String notes = limelight == null
                ? "; no camera -- no \"limelight\" configured" : describeIgnored();
        Pose expected = expectedTarget();
        if (expected == null) {
            return "no data" + notes;
        }
        String target = memory.isEmpty() ? "nearest CELL (field map)" : "remembered HIVE";
        String state = reachedExpectedHeading
                ? "ON HEADING to " + target + " -- no tags in view, gamepad 1 turns"
                : aimHeld ? "TURNING to " + target
                : target;
        Pose robot = follower.getPose();
        double headingDeg = Utils.degrees(robot.getHeading());
        double targetDeg = Utils.degrees(
                robot.getHeading() + Math.toRadians(headingErrorTo(robot, expected)));
        return String.format(Locale.US, "%s, heading %.0f° -> %.0f°%s",
                state, headingDeg, targetDeg, notes);
    }

    /**
     * Where the remembered HIVE is, for telemetry: how far away and which way to turn to face it
     * from where the robot is now, and how long ago tags last updated it. Before any tags have
     * been seen, the same for the field map's nearest CELL.
     */
    public String describeHive() {
        if (!memory.isEmpty()) {
            return String.format(Locale.US, "%s, seen %.1fs ago",
                    describeFromRobot(memory.getPosition()), memory.secondsSinceSeen());
        }
        String notSeen = limelight == null ? "no camera"
                : "not seen yet (needs tags in view, with Full 3D on in the Limelight)";
        Pose cell = nearestMapCell();
        return cell == null ? notSeen
                : notSeen + "; nearest CELL on field map " + describeFromRobot(cell);
    }

    /** How far away point is and which way to turn to face it, e.g. "4'8\" away, 12° left". */
    private String describeFromRobot(Pose point) {
        Pose robot = follower.getPose();
        double turnDeg = headingErrorTo(robot, point);  // positive is counterclockwise (left)
        long inches = Math.round(robot.distanceFrom(point));
        return String.format(Locale.US, "%d'%d\" away, %.0f° %s",
                inches / 12, inches % 12, Math.abs(turnDeg), turnDeg >= 0 ? "left" : "right");
    }

    /** A line per visible alliance tag, for debugging; see TagDebugTelemetry. */
    public void addTagTelemetry(Telemetry telemetry) {
        TagDebugTelemetry.add(telemetry, aimTags, upsideDownTags, flatTags);
    }

    /**
     * Notes for the end of the status line: which tags were left out and why, e.g.
     * "; ignoring upside-down 31, 33; ignoring flat 30, 32", and a warning when tags arrive
     * without the corners or 3D pose needed to check them. "" when there's nothing to note.
     */
    private String describeIgnored() {
        StringBuilder result = new StringBuilder();
        appendIgnored(result, "upside-down", upsideDownTags);
        appendIgnored(result, "flat", flatTags);
        if (cornersMissing) {
            result.append("; NO CORNERS from Limelight -- can't spot upside-down tags");
        }
        if (tiltMissing) {
            result.append("; NO 3D from Limelight -- can't spot flat tags");
        }
        return result.toString();
    }

    /** Appends e.g. "; ignoring flat 30, 32", or nothing when tags is empty. */
    private static void appendIgnored(StringBuilder result, String why,
                                      List<LLResultTypes.FiducialResult> tags) {
        if (tags.isEmpty()) {
            return;
        }
        result.append("; ignoring ").append(why);
        for (int i = 0; i < tags.size(); i++) {
            result.append(i == 0 ? " " : ", ").append(tags.get(i).getFiducialId());
        }
    }

    private static boolean isAllianceTag(int id, boolean redAlliance) {
        return redAlliance
                ? RED_TAG_MIN_ID <= id && id <= RED_TAG_MAX_ID
                : BLUE_TAG_MIN_ID <= id && id <= BLUE_TAG_MAX_ID;
    }
}
