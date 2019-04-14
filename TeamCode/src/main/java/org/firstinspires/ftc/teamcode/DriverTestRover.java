package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestDriver", group = "Main")

public class DriverTestRover extends DriverRover {

    boolean doneRotating = false;

    @Override
    public void loop() {

        if (gamepad2.b) {
            if (!doneRotating) {
                doneRotating = rotateToHeading(getHeadingParallelToWall());
            } else {
                doArc(ourCrater ? 0.45 : -0.45);
            }
        } else if (gamepad2.x) {
            if (!doneRotating) {
                doneRotating = rotateToHeading(getHeadingAtDepotLander());
            } else {
                doArc(ourCrater ? -0.45 : 0.45);
            }
        } else if (gamepad2.a) {
            doneRotating = false;
            rotateToHeading(getHeadingAtDepotLander());
        } else {
            wheels.powerMotors(0, 0, 0);
        }



    }
}
