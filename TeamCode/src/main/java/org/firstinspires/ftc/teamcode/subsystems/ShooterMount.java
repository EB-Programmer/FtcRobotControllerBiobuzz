package org.firstinspires.ftc.teamcode.subsystems;

/**
 * The robot's two shooters: where each sits relative to the robot's center, and which way they
 * shoot. Both shoot straight out the robot's back, parallel to its heading, so aiming means
 * turning the back toward the HIVE. Each shooter is also off to one side of the center, so
 * pointing the robot's center line at the HIVE would send the ball past it on that shooter's
 * side; aiming a shooter turns a little further, until the HIVE is straight behind that shooter
 * (see headingOffset()).
 *
 * Both shooters are 5 in behind the robot's center. That doesn't change the aim: sliding a
 * shooter along the direction it shoots leaves its line of fire where it is.
 */
public enum ShooterMount {
    POLLEN("pollen", -4.75),
    NECTAR("nectar", 2.75);

    // Which way both shooters shoot, counterclockwise from the robot's front: 180 is out the back
    public static final double SHOOTING_DIRECTION_DEG = 180;

    public final String label;
    // Left of the robot's center, in inches (negative is right)
    public final double leftInches;

    ShooterMount(String label, double leftInches) {
        this.label = label;
        this.leftInches = leftInches;
    }

    /**
     * Radians to add to the heading that points the robot's front at a target, positive
     * counterclockwise, so that this shooter points at it instead. distance is how far the target
     * is from the robot's center, in inches. Without a usable distance -- NaN, or a target closer
     * than the shooter is to the side, where no heading lines the shooter up with it -- only the
     * shooting direction is allowed for, as with centerLineOffset().
     */
    public double headingOffset(double distance) {
        double direction = Math.toRadians(SHOOTING_DIRECTION_DEG);
        // How far the shooter sits to the left of the line it shoots along through the robot's
        // center: shooting backward, the robot's right is the shot's left
        double sideways = leftInches * Math.cos(direction);
        if (!(distance > Math.abs(sideways))) {
            return centerLineOffset();
        }
        // The robot turns about its center, so the target stays this far away whichever way it
        // faces. The right heading puts the target that far to the side of the robot's center
        // line, which is straight along the shooter's line of fire.
        return -direction - Math.asin(sideways / distance);
    }

    /**
     * Radians to add to the heading that points the robot's front at a target so that the robot's
     * center line points at it the way the shooters shoot, with no allowance for either
     * shooter's offset to the side.
     */
    public static double centerLineOffset() {
        return -Math.toRadians(SHOOTING_DIRECTION_DEG);
    }
}
