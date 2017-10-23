package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by JASMINE on 10/22/17.
 * This class has the code for our driver controlled mode.
 */
@TeleOp(name="DriverOpMode", group ="Main")
public class DriverOpMode_Relic extends OpMode {

    private MecanumWheels mecanumWheels;
    private boolean backButtonPressed;

    @Override
    public void init() {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry);
        mecanumWheels.powerMotors(0, 0, 0);

    }

    @Override
    public void start() {
        backButtonPressed = false;
    }

    @Override
    public void loop() {


        //Changing direction from forward to backward and backward to forward
        //Gamepad back button does not work with motorola 3G
        if (gamepad1.a) {
            backButtonPressed = true;
        } else if (backButtonPressed) {
            backButtonPressed = false;
            mecanumWheels.changeDirection();
            //todo: add lights
        }


        double clockwise = gamepad1.right_stick_x;


        //We are using robot coordinates
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            double forward = 0;
            double right = 0;
            if (gamepad1.dpad_up) {
                forward = 1;
            } else if (gamepad1.dpad_down) {
                forward = -1;
            }
            if (gamepad1.dpad_right) {
                right = 1;
            } else if (gamepad1.dpad_left) {
                right = -1;
            }
            mecanumWheels.powerMotors(forward, right, clockwise);
        } else {
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            mecanumWheels.powerMotors(forward, right, clockwise);
        }

    }

    //todo
//    /**
//     * @param inPos - servo all the way in position
//     * @param outPos - servo all the way out position
//     * @param percentOpen - if inPos is 0.0 and outPos is 1.0
//     * @return
//     */
//    public static double calculateServoPosition(double inPos, double outPos, double percentOpen){
//
//    }


}
