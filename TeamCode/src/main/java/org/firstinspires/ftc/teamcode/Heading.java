package org.firstinspires.ftc.teamcode;

// Use Math.cos, Math.sin Math.toRadians, Math.toDegrees, Math
public class Heading {
    double cos;
    double sin;

    /**
     * represents an angle by its cos and sin values, aka the rectangular coordinates
     * @param angle is the gyro angle, so it is in degrees
     */
    public Heading(double angle) {
        double angleInRad = Math.toRadians(angle);
        cos = Math.cos(angleInRad);
        sin = Math.sin(angleInRad);
    }

    /**
     * returns the angle in degrees that the robot should rotate clockwise
     * @param startHeading beginning heading
     * @param targetHeading ending heading
     */
    public static double clockwiseRotateAngle(Heading startHeading, Heading targetHeading) {
        double sine = Math.sin(startHeading.sin * targetHeading.cos - startHeading.cos * targetHeading.sin);
        double cosine = Math.cos(startHeading.cos * targetHeading.cos + startHeading.sin * targetHeading.sin);

        return Math.toDegrees(Math.atan2(sine, cosine));
    }
}
