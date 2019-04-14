package org.firstinspires.ftc.teamcode;

// Use Math.cos, Math.sin Math.toRadians, Math.toDegrees, Math.toRadians

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
     * @param startHeading beginning heading
     * @param targetHeading ending heading
     * @return the angle in degrees that the robot should rotate clockwise
     */
    public static double clockwiseRotateAngle(Heading startHeading, Heading targetHeading) {
        double sine = startHeading.sin * targetHeading.cos - startHeading.cos * targetHeading.sin;
        double cosine = startHeading.cos * targetHeading.cos + startHeading.sin * targetHeading.sin;

        return Math.toDegrees(Math.atan2(sine, cosine));
    }
}
