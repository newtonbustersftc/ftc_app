package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "DriverRover", group = "Main")
public class DriverRover extends OpMode {
    //TODO: EDIT LATER ON
    private static final double MINPOWER = 0.1;

    private DcMotor motorLeft;
    private DcMotor motorRight;

    //for now, we don't support robot reversing the forward direction
    private final boolean forward = true;

    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("wheelsLeft");
        motorRight = hardwareMap.dcMotor.get("wheelsRight");

        // run by power
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // float zero power
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //power given to the left side wheels, power given to the right side wheels
        //forward as a direction
        double leftForward;
        double rightForward;

        int sign = forward ? 1 : -1;
        if (!gamepad1.y) {
            //Arcade Drive
            rightForward = -(scaled(gamepad1.left_stick_y) + sign * scaled(gamepad1.left_stick_x));
            leftForward = -(scaled(gamepad1.left_stick_y) - sign * scaled(gamepad1.left_stick_x));
        } else {
            //Tank Drive
            leftForward = -scaled(gamepad1.left_stick_y);
            rightForward = -scaled(gamepad1.right_stick_y);
        }

        if ((gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right)) {
            // dPadDrive: Forward, Backwards, Rotate in place CClockwise and Clockwise
            //We are using robot coordinates
            double dpadSpeed = 0.2;
            if (gamepad1.dpad_up) {
                rightForward = dpadSpeed;
                leftForward = dpadSpeed;
            } else if (gamepad1.dpad_down) {
                rightForward = -dpadSpeed;
                leftForward = -dpadSpeed;
            } else if (gamepad1.dpad_left) {
                //rotating ccw
                rightForward = sign * dpadSpeed;
                leftForward = -sign * dpadSpeed;
            } else {
                //rotating cw
                leftForward = sign * dpadSpeed;
                rightForward = -sign * dpadSpeed;
            }
        }

        //driving backwards
        if (!forward) { //when start, front direction is the intake side, lightStrip2
            leftForward = -leftForward;
            rightForward = -rightForward;
        }

        //todo adjust the deadband
        if ((rightForward > -0.01) && (rightForward < 0.01))
            rightForward = 0;
        else if ((rightForward > -MINPOWER) && (rightForward < MINPOWER))
            rightForward = MINPOWER * rightForward / Math.abs(rightForward);

        if ((leftForward > -0.01) && (leftForward < 0.01))
            leftForward = 0;
        else if ((leftForward > -MINPOWER) && (leftForward < MINPOWER))
            leftForward = MINPOWER * leftForward / Math.abs(leftForward);

        rightForward = Range.clip(rightForward, -1, 1);
        leftForward = Range.clip(leftForward, -1, 1);

        telemetry.addData("left", leftForward);
        telemetry.addData("right", rightForward);

        powerMotors(rightForward, leftForward);
    }

    private void powerMotors(double rightForward, double leftForward) {
        motorLeft.setPower(leftForward);
        motorRight.setPower(rightForward);
    }

    private double scaled (double x) {
        return (x / 1.07) * (.62 * x * x + .45);
    }
}