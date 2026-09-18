package org.firstinspires.ftc.teamcode.examples;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates the Pedro Pathing Telemetry menu system (com.pedropathing:telemetry).
 * On init, drive the cursor with the d-pad, press right bumper to select /
 * drill into a folder, left bumper to go back. Once a leaf item is chosen,
 * it becomes the OpMode that actually runs after START is pressed.
 */
@Autonomous(name = "EB Auton: Menu Example", group="EBBiobuzzTest")
public class ExampleMenuAuton extends SelectableOpMode {

    public ExampleMenuAuton() {
        super("Choose Autonomous", scope -> {
            scope.folder("Start: Left", left -> {
                left.add("Delay 0s", () -> new DriveForwardAuton(0));
                left.add("Delay 2s", () -> new DriveForwardAuton(2));
            });
            scope.folder("Start: Right", right -> {
                right.add("Delay 0s", () -> new DriveForwardAuton(0));
                right.add("Delay 2s", () -> new DriveForwardAuton(2));
            });
            scope.add("Do Nothing", DoNothingAuton::new);
        });
    }

    /**
     * Placeholder routine: waits out the chosen delay, then "drives" for a
     * couple seconds. Swap the telemetry lines for real Follower/motor calls.
     */
    private static class DriveForwardAuton extends OpMode {
        private final int delaySeconds;
        private final ElapsedTime timer = new ElapsedTime();

        DriveForwardAuton(int delaySeconds) {
            this.delaySeconds = delaySeconds;
        }

        @Override
        public void init() {
            telemetry.addData("Selected delay", delaySeconds + "s");
        }

        @Override
        public void start() {
            timer.reset();
        }

        @Override
        public void loop() {
            if (timer.seconds() < delaySeconds) {
                telemetry.addLine("Waiting out delay...");
            } else if (timer.seconds() < delaySeconds + 2) {
                telemetry.addLine("Driving forward!");
                // TODO: replace with real drivetrain / Follower movement
            } else {
                telemetry.addLine("Done.");
            }
            telemetry.addData("Elapsed", "%.1fs", timer.seconds());
        }
    }

    private static class DoNothingAuton extends OpMode {
        @Override
        public void init() {
            telemetry.addLine("Doing nothing this match.");
        }

        @Override
        public void loop() {
        }
    }
}
