package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Aims the robot at its alliance's HIVE using the Limelight's view of the AprilTags on the HIVE
 * CELLS. Which CELL faces up changes after each HIVE TIP, so rather than tracking one tag, this
 * averages the horizontal angle to every visible tag from our alliance.
 */
public class HiveAim {
    // AprilTag IDs on the HIVE CELLS: red 30-37, blue 38-45
    private static final int RED_TAG_MIN_ID = 30;
    private static final int RED_TAG_MAX_ID = 37;
    private static final int BLUE_TAG_MIN_ID = 38;
    private static final int BLUE_TAG_MAX_ID = 45;
    private static final int APRILTAG_PIPELINE = 4;  // Limelight pipeline slot
    private static final double KP = 0.02;  // turn power per degree of tx
    private static final double MAX_TURN = 0.5;
    private static final double TOLERANCE_DEG = 1.0;
    private static final long MAX_RESULT_STALENESS = 100;  // milliseconds

    private final Limelight3A limelight;
    private final List<LLResultTypes.FiducialResult> allianceTags = new ArrayList<>();
    private int tagCount = 0;
    private double tx = 0;

    public HiveAim(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);  // how many times/sec to request data
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
    }

    /** Starts the Limelight sending results; call before update(). */
    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    /** Re-reads the Limelight's latest result, keeping only tags from the given alliance. */
    public void update(boolean redAlliance) {
        allianceTags.clear();
        tagCount = 0;
        tx = 0;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() >= MAX_RESULT_STALENESS) {
            return;
        }
        double txSum = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (isAllianceTag(tag.getFiducialId(), redAlliance)) {
                allianceTags.add(tag);
                txSum += tag.getTargetXDegrees();
                tagCount++;
            }
        }
        if (tagCount > 0) {
            tx = txSum / tagCount;
        }
    }

    public boolean hasTarget() {
        return tagCount > 0;
    }

    public int getTagCount() {
        return tagCount;
    }

    /** Average horizontal angle to the visible tags; positive means they're to the right. */
    public double getTx() {
        return tx;
    }

    /**
     * Turn power that points the robot at the visible tags, in Pedro Pathing's convention
     * (positive turns counterclockwise), or 0 with no target or once within tolerance.
     */
    public double getTurn() {
        if (!hasTarget() || Math.abs(tx) < TOLERANCE_DEG) {
            return 0;
        }
        // Tags to the right (positive tx) need a clockwise turn, which is negative here
        return -Math.max(-MAX_TURN, Math.min(MAX_TURN, KP * tx));
    }

    /**
     * Adds a line per visible alliance tag with everything that might tell an upright tag from an
     * upside-down one: its position in the image, its 3D orientation relative to the camera, and
     * the image direction from its first corner to its second.
     */
    public void addTagTelemetry(Telemetry telemetry) {
        for (LLResultTypes.FiducialResult tag : allianceTags) {
            telemetry.addData("Tag " + tag.getFiducialId(), "tx %.1f ty %.1f area %.2f | %s | %s",
                    tag.getTargetXDegrees(), tag.getTargetYDegrees(), tag.getTargetArea(),
                    describeOrientation(tag.getTargetPoseCameraSpace()),
                    describeCorners(tag.getTargetCorners()));
        }
    }

    private static String describeOrientation(Pose3D pose) {
        // The SDK fills in all zeros when the Limelight sends no 3D pose, which would otherwise
        // look just like a perfectly upright tag
        Position position = pose.getPosition();
        if (position.x == 0 && position.y == 0 && position.z == 0) {
            return "3D n/a";
        }
        YawPitchRollAngles angles = pose.getOrientation();
        return String.format(Locale.US, "ypr %.0f/%.0f/%.0f",
                angles.getYaw(AngleUnit.DEGREES), angles.getPitch(AngleUnit.DEGREES),
                angles.getRoll(AngleUnit.DEGREES));
    }

    private static String describeCorners(List<List<Double>> corners) {
        if (corners.size() < 2) {
            return "no corners";
        }
        double dx = corners.get(1).get(0) - corners.get(0).get(0);
        double dy = corners.get(1).get(1) - corners.get(0).get(1);
        return String.format(Locale.US, "c0->1 %.0f°", Math.toDegrees(Math.atan2(dy, dx)));
    }

    private static boolean isAllianceTag(int id, boolean redAlliance) {
        return redAlliance
                ? RED_TAG_MIN_ID <= id && id <= RED_TAG_MAX_ID
                : BLUE_TAG_MIN_ID <= id && id <= BLUE_TAG_MAX_ID;
    }
}
