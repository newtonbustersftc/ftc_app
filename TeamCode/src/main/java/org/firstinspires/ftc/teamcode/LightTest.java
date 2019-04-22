package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "LightTest", group = "Sensor")
public class LightTest extends OpMode {

    Servo[] lights = new Servo[3];
    final int RED = 0;
    final int GREEN = 1;
    final int BLUE = 2;

    @Override
    public void init() {
        for(int i = 1;i <= 3; i++) {
            int lightIndex = i - 1;
            lights[lightIndex] = hardwareMap.servo.get("light" + i);
            DriverRover.setUpServo(lights[lightIndex], 0.5, 1);
        }
    }

    @Override
    public void loop() {
        double minVal = 0.05;
        if(gamepad1.right_stick_x > minVal) {
            lights[RED].setPosition(gamepad1.right_stick_x);
        } else {
            lights[RED].setPosition(0);
        }

        if(gamepad1.left_stick_x > minVal) {
            lights[GREEN].setPosition(gamepad1.left_stick_x);
        } else {
            lights[GREEN].setPosition(0);
        }

        if(gamepad1.right_trigger > minVal) {
            lights[BLUE].setPosition(gamepad1.right_trigger);
        } else {
            lights[BLUE].setPosition(0);
        }
    }
}
