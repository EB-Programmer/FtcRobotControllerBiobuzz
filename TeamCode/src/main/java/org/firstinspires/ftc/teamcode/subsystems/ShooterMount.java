package org.firstinspires.ftc.teamcode.subsystems;

/**
 * The robot's two shooters and how far each sits to the side of the robot's center. Both shoot
 * straight ahead, parallel to the robot's heading, so pointing the robot's center at the HIVE
 * would send the ball past it on the shooter's side. Aiming a shooter instead means turning a
 * little toward that shooter's side, until the HIVE is straight ahead of the shooter (see
 * headingOffset()).
 *
 * Both shooters are also 5 in behind the robot's center. That doesn't change the aim: sliding a
 * shooter along the direction it shoots leaves its line of fire where it is.
 */
public enum ShooterMount {
    POLLEN("pollen", -4.75),
    NECTAR("nectar", 2.75);

    public final String label;
    // Left of the robot's center, in inches (negative is right)
    public final double leftInches;

    ShooterMount(String label, double leftInches) {
        this.label = label;
        this.leftInches = leftInches;
    }

    /**
     * Radians to add to the heading that points the robot's center at a target, positive
     * counterclockwise, so that this shooter points at it instead. distance is how far the target
     * is from the robot's center, in inches. 0 when it isn't known (NaN) or the target is closer
     * than the shooter is to the side, where no heading lines the shooter up with it.
     */
    public double headingOffset(double distance) {
        if (!(distance > Math.abs(leftInches))) {
            return 0;
        }
        // The robot turns about its center, so the target stays this far away whichever way it
        // faces. The right heading puts the target leftInches left of the robot's center line,
        // which is straight ahead of the shooter.
        return -Math.asin(leftInches / distance);
    }
}
