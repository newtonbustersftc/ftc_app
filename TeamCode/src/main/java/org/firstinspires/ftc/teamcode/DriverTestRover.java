package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestDriver", group = "Main")

public class DriverTestRover extends DriverRover {

    boolean doneRotating = false;
    boolean bPressed = false;
    boolean aPressed = false;

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                doArc(gamepad1.right_stick_x);
            } else if (gamepad1.b) {
                if(!bPressed) {
                    bPressed = true;
                    arcstate = ArcState.AT_CRATER;
                }
                toLauncher();
                //Align robot along the wall
                //rotateToHeading(getHeadingParallelToWall());
            } else if (gamepad1.a) {
                if(!aPressed) {
                    aPressed = true;
                    arcstate = ArcState.AT_LAUNCHER;
                }
                toCrater();
                //Align robot perpendicular to depot launcher
                //rotateToHeading(getHeadingAtDepotLander());
            } else if (gamepad1.x) {
                //Align robot perpendicular to crater launcher
                rotateToHeading(getHeadingAtCraterLander());
            }
        } else {
            controlWheels();
        }
        if(!gamepad1.b) {
            bPressed = false;
        }
        if(!gamepad1.a) {
            aPressed = false;
        }
    telemetry.addData("ArcState", arcstate);
    }
}
