package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Locale;

/**
 * Per-tag telemetry lines for checking what the camera sees: each line shows how aiming used the
 * tag, plus its position and size in the image, its 3D rotation relative to the camera (pitch is
 * the tilt TagTilt uses to spot flat tags), and the corner edge angle that TagCorners uses to spot
 * upside-down tags.
 */
final class TagDebugTelemetry {
    private TagDebugTelemetry() {
    }

    static void add(Telemetry telemetry, TagCluster aimTags,
                    List<LLResultTypes.FiducialResult> upsideDownTags,
                    List<LLResultTypes.FiducialResult> flatTags) {
        for (LLResultTypes.FiducialResult tag : aimTags.getTags()) {
            addTag(telemetry, tag, "aim");
        }
        for (LLResultTypes.FiducialResult tag : upsideDownTags) {
            addTag(telemetry, tag, "upside down");
        }
        for (LLResultTypes.FiducialResult tag : flatTags) {
            addTag(telemetry, tag, "flat");
        }
    }

    private static void addTag(Telemetry telemetry, LLResultTypes.FiducialResult tag,
                               String role) {
        // getTargetArea() is the fraction of the image the tag covers; shown as a percent
        telemetry.addData("Tag " + tag.getFiducialId() + " (" + role + ")",
                "tx %.1f ty %.1f area %.2f%% | %s | %s",
                tag.getTargetXDegrees(), tag.getTargetYDegrees(), tag.getTargetArea() * 100,
                describeRotation(tag.getTargetPoseCameraSpace()),
                describeCorners(tag));
    }

    /**
     * The tag's rotation in the Limelight's own terms: roll is its twist in the picture (near 0
     * upright, near ±180 upside down), yaw is turning left/right, pitch is tilting toward/away.
     */
    private static String describeRotation(Pose3D pose) {
        // Checked first: all zeros would otherwise look just like a perfectly upright tag
        if (!LimelightMount.has3D(pose)) {
            return "3D n/a";
        }
        // The Limelight sends camera-space rotation as pitch, yaw, roll, but the SDK files those
        // three under roll, pitch, yaw -- so the SDK's yaw holds the Limelight's roll, its pitch
        // the Limelight's yaw, and its roll the Limelight's pitch
        YawPitchRollAngles angles = pose.getOrientation();
        return String.format(Locale.US, "roll %.0f yaw %.0f pitch %.0f",
                angles.getYaw(AngleUnit.DEGREES), angles.getPitch(AngleUnit.DEGREES),
                angles.getRoll(AngleUnit.DEGREES));
    }

    private static String describeCorners(LLResultTypes.FiducialResult tag) {
        if (!TagCorners.hasCorners(tag)) {
            return "no corners";
        }
        return String.format(Locale.US, "edge %.0f°", TagCorners.edgeAngleDeg(tag));
    }
}
