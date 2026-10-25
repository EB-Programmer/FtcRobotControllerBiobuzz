package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Locale;

/**
 * Per-tag telemetry lines for working out a way to tell upright tags from upside-down ones beyond
 * TagCluster's tag-order check: each line shows how aiming used the tag, plus its position in the
 * image, its 3D orientation relative to the camera, and the image direction from its first corner
 * to its second.
 */
final class TagDebugTelemetry {
    private TagDebugTelemetry() {
    }

    static void add(Telemetry telemetry, List<TagCluster> clusters, TagCluster aimCluster) {
        for (TagCluster cluster : clusters) {
            String role = cluster == aimCluster ? "aim"
                    : cluster.orientation() == TagCluster.Orientation.DOWN ? "upside down"
                    : "not used";
            for (LLResultTypes.FiducialResult tag : cluster.getTags()) {
                telemetry.addData("Tag " + tag.getFiducialId() + " (" + role + ")",
                        "tx %.1f ty %.1f area %.2f | %s | %s",
                        tag.getTargetXDegrees(), tag.getTargetYDegrees(), tag.getTargetArea(),
                        describeOrientation(tag.getTargetPoseCameraSpace()),
                        describeCorners(tag.getTargetCorners()));
            }
        }
    }

    private static String describeOrientation(Pose3D pose) {
        // Checked first: all zeros would otherwise look just like a perfectly upright tag
        if (!LimelightMount.has3D(pose)) {
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
}
