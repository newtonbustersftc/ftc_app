package org.firstinspires.ftc.teamcode;

/**
 *
 */
interface ErrorSource {

    /**
     * error is positive if we want clockwise correction, negative if we want counterclockwise
     * @return
     */
    public double getError();

    /**
     * KP = proportional correction coefficient
     * @return
     */
    public double getKP();
}
