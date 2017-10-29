package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class is controlling the mecanum wheels driving platform.
 * Created by JASMINE on 10/22/17.
 */

class MecanumWheels {

    //defining the 4 motors
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorRearLeft;
    private DcMotor motorRearRight;
    private Telemetry telemetry;
    private boolean forward;


    MecanumWheels(HardwareMap hardwareMap, Telemetry telemetry, boolean frontForward) {
        this.telemetry = telemetry;
        this.forward = frontForward;

        //"initializing" the motors
        motorFrontLeft = hardwareMap.dcMotor.get("Front-Left"); //DC1
        motorFrontRight = hardwareMap.dcMotor.get("Front-Right");//DC2
        motorRearLeft = hardwareMap.dcMotor.get("Rear-Left"); //DC3
        motorRearRight = hardwareMap.dcMotor.get("Rear-Right"); //DC4

        //setting the motors on the left side in reverse so both wheels spin the same way.
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

    }

    MecanumWheels(HardwareMap hardwareMap, Telemetry telemetry){
        this(hardwareMap, telemetry, true);

    }

    void changeDirection(){
        forward = !forward;

    }


    void powerMotors(double forward, double right, double clockwise) {

        //add deadband so you don't strafe when you don't want to. A deadband is essentially if you want to go to the right,
        //and the joystick is 7 degrees short of 90 degrees, instead of having the robot slowly creep forward, the robot will
        //ignore the small degrees and just go to the right.
        //todo adjust the deadband
        if ((right > -0.1) && (right < 0.1)) right = 0;
        if ((forward > -0.1) && (forward < 0.1)) forward = 0;

        if (this.forward){
            forward = - forward;
            right =  - right;
        }


        double front_left = forward + clockwise + right;
        double front_right = forward - clockwise - right;
        double rear_left = forward + clockwise - right;
        double rear_right = forward - clockwise + right;

        /*this is scaling the motor power. Since our motors work on a scale between -1 and 1, and when we input
        values into the controller, they can be greater than one. We want to make sure that all values are between 1 and -1.
         we do that by first figuring out what the maximum value is, and then dividing all the numbers by the max value. Therefore
        the max power will be 1 (or -1, if we are going in reverse) and the other powers will be less than one. */
        //todo find the maximum absolute value for any motor scaled power
        double max = Math.abs(front_left);
        max = Math.max(Math.abs(front_right), max);
        max = Math.max(Math.abs(rear_left), max);
        max = Math.max(Math.abs(rear_right), max);

        if (max > 1) {
            front_left /= max;
            front_right /= max;
            rear_left /= max;
            rear_right /= max;
        }

        /* assigning the motors the scaled powers that we just calculated in the step above. */
        motorFrontLeft.setPower(front_left);
        motorFrontRight.setPower(front_right);
        motorRearLeft.setPower(rear_left);
        motorRearRight.setPower(rear_right);

        // Telemetry - all doubles are scaled to (-100, 100)
        /*
        telemetry.addData("Robot Forward,Right,Clockwise", (int) (forward * 100) +
                ", " + (int) (right * 100)+
                ", " + (int) (clockwise * 100));
        */
        telemetry.addData("DC1,2,3,4", (int) (front_left * 100) + ", " +
                (int) (front_right * 100) + ", " +
                (int) (rear_left * 100) + ", " +
                (int) (rear_right * 100));
        telemetry.update();
    }

    private void setRunMode(DcMotor.RunMode runMode) {
        motorFrontLeft.setMode(runMode);
        motorFrontRight.setMode(runMode);
        motorRearLeft.setMode(runMode);
        motorRearRight.setMode(runMode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {

        motorFrontLeft.setZeroPowerBehavior(behavior);
        motorFrontRight.setZeroPowerBehavior(behavior);
        motorRearLeft.setZeroPowerBehavior(behavior);
        motorRearRight.setZeroPowerBehavior(behavior);

    }

    //-----------------------
    //AUTONOMOUS MODE METHODS
    //-----------------------



    private void resetEncoders() {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        String message = "Reset encoders did not complete";

        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            message += " " + e.getMessage();
            telemetry.addData("ERROR", message);
        }

        while (Math.abs(motorFrontLeft.getCurrentPosition()) > 0 ||
                Math.abs(motorFrontRight.getCurrentPosition()) > 0 ||
                Math.abs(motorRearLeft.getCurrentPosition()) > 0 ||
                Math.abs(motorRearRight.getCurrentPosition()) > 0) {
            telemetry.addData("-PosFL,FR,RL,RR ",
                    motorFrontLeft.getCurrentPosition() + "," +
                    motorFrontRight.getCurrentPosition() + "," +
                    motorRearLeft.getCurrentPosition() + "," +
                    motorRearRight.getCurrentPosition());
            telemetry.update();
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                message += " " + e.getMessage();
                telemetry.addData("ERROR", message);
            }

        }

        logEncoders();
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void logEncoders() {
        telemetry.addData("PosFL,FR,RL,RR", motorFrontLeft.getCurrentPosition() + "," +
                motorFrontRight.getCurrentPosition() + "," +
                motorRearLeft.getCurrentPosition() + "," +
                motorRearRight.getCurrentPosition());
        telemetry.update();
    }

    public void goCounts(int counts) throws InterruptedException {
        resetEncoders();
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int startPos = motorFrontRight.getCurrentPosition();
        powerMotors(0.4, 0, 0);
        int currentPos = startPos;
        while ( Math.abs(currentPos-startPos) < counts ) {
            Thread.yield();
            logEncoders();
            currentPos = motorFrontRight.getCurrentPosition();
        }
        powerMotors(0,0,0);
        logEncoders();
    }

}