package org.firstinspires.ftc.teamcode.RelicRecovery;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class is controlling the mecanum wheels driving platform.
 * Created by JASMINE on 10/22/17.
 */

class MecanumWheels {

    public enum Wheel {FL, FR, RL, RR}

    public static final double MIN_FORWARD = 0.2; //minimum power to move the robot forward
    public static final double MIN_RIGHT = 0.4; //minimum power to strafe
    public static final double MIN_CLOCKWISE = 0.25; //minimum power to rotate

    //defining the 4 motors
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorRearLeft;
    private DcMotor motorRearRight;
    private Telemetry telemetry;
    private boolean forward;

    private final String REVERSED = "REVERSED";

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

    MecanumWheels(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, true);
    }

    void changeDirection() {
        forward = !forward;
    }
    boolean isForward() {return forward;}

    void powerMotors(double forward, double right, double clockwise) {
        powerMotors(forward, right, clockwise, this.forward);
    }

    void powerMotors(double forward, double right, double clockwise, boolean forwardDir) {

//        telemetry.addData("Gamepad Forward,Right,Clockwise", (int) (forward * 100) +
//                ", " + (int) (right * 100) +
//                ", " + (int) (clockwise * 100));

        double rightSignFactor = right > 0 ? 1 : -1;
        double forwardSignFactor = forward > 0 ? 1 : -1;
        double clockwiseSignFactor = clockwise > 0 ? 1 : -1;

        //apply deadbands
        right = adjustpower(right, MIN_RIGHT) * rightSignFactor;
        forward = adjustpower(forward, MIN_FORWARD) * forwardSignFactor;
        clockwise = adjustpower(clockwise, MIN_CLOCKWISE) * clockwiseSignFactor;

        if (forwardDir) {
            forward = -forward;
            right = -right;
        }

        double front_left = forward - clockwise + right;
        double front_right = forward + clockwise - right;
        double rear_left = forward - clockwise - right;
        double rear_right = forward + clockwise + right;

        //scale motor power to be between -1 and 1
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

        // assign the motors the scaled powers
        motorFrontLeft.setPower(front_left);
        motorFrontRight.setPower(front_right);
        motorRearLeft.setPower(rear_left);
        motorRearRight.setPower(rear_right);

        // Telemetry - all doubles are scaled to (-100, 100)
        if (!isForward()) {
            telemetry.addData("Driver dir", REVERSED);
        }
        telemetry.addData("DC1,2,3,4", (int) (front_left * 100) + ", " +
                (int) (front_right * 100) + ", " +
                (int) (rear_left * 100) + ", " +
                (int) (rear_right * 100));
    }

    double MIN_POWER_GAMEPAD = 0.3;

    public double adjustpower(double padPower, double minPower) {
        if (Math.abs(padPower) < 0.05) return 0;
        if (Math.abs(padPower) < MIN_POWER_GAMEPAD) return minPower;
        // linear for rest, from minPower to 1.0
        return ((1 - minPower) / (1 - MIN_POWER_GAMEPAD) * (Math.abs(padPower) - 1) + 1);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        motorFrontLeft.setMode(runMode);
        motorFrontRight.setMode(runMode);
        motorRearLeft.setMode(runMode);
        motorRearRight.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {

        motorFrontLeft.setZeroPowerBehavior(behavior);
        motorFrontRight.setZeroPowerBehavior(behavior);
        motorRearLeft.setZeroPowerBehavior(behavior);
        motorRearRight.setZeroPowerBehavior(behavior);

    }

    //-----------------------
    //AUTONOMOUS MODE METHODS
    //-----------------------


    public void resetEncoders() {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        long startMillis = System.currentTimeMillis();

        String message = "Reset encoders did not complete";
        int tolerance = 10;
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            message += " " + e.getMessage();
            telemetry.addData("ERROR", message);
        }

        while (Math.abs(motorFrontLeft.getCurrentPosition()) > tolerance ||
                Math.abs(motorFrontRight.getCurrentPosition()) > tolerance ||
                Math.abs(motorRearLeft.getCurrentPosition()) > tolerance ||
                Math.abs(motorRearRight.getCurrentPosition()) > tolerance) {
            telemetry.addData("-PosFL,FR,RL,RR ",
                    motorFrontLeft.getCurrentPosition() + "," +
                            motorFrontRight.getCurrentPosition() + "," +
                            motorRearLeft.getCurrentPosition() + "," +
                            motorRearRight.getCurrentPosition());
            telemetry.update();
            if (System.currentTimeMillis() - startMillis > 2000) {
                break; // time out
            }
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

    public void logEncoders() {
        telemetry.addData("PosFL,FR,RL,RR", motorFrontLeft.getCurrentPosition() + "," +
                motorFrontRight.getCurrentPosition() + "," +
                motorRearLeft.getCurrentPosition() + "," +
                motorRearRight.getCurrentPosition());
        telemetry.update();
    }

    public DcMotor getMotor(Wheel wheel) {
        switch (wheel) {
            case FR:
                return motorFrontRight;
            case FL:
                return motorFrontLeft;
            case RR:
                return motorRearRight;
            case RL:
                return motorRearLeft;
            default:
                return null;
        }
    }


}