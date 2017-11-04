package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by JASMINE on 10/22/17.
 * This class has the code for our driver controlled mode.
 */
@TeleOp(name = "DriverOpMode", group = "Main")
public class DriverOpMode_Relic extends OpMode {

    double MAX_DRIVE_POWER = 0.3;

    private MecanumWheels mecanumWheels;
    private boolean backButtonPressed;

    private DcMotor lift; //DcMotor for the lift
    private int LIFT_COUNT_MAX = 6000;

    private boolean touchSensorReleased;

    private DigitalChannel touchSensor; //Touch sensor at lowest position on the lift

    Servo leftHand;
    public static final double LEFT_HAND_IN_POS = 0.63;
    public static final double LEFT_HAND_OUT_POS = 0.48;

    Servo rightHand;
    public static final double RIGHT_HAND_IN_POS = 0.20;
    public static final double RIGHT_HAND_OUT_POS = 0.4;

    Servo jewelArm;
    public static final double JEWEL_ARM_HOME = 0.72; // home position
    public static final double JEWEL_ARM_DOWN = 0.02; // down position
    public static final double JEWEL_ARM_VERTICAL = 0.55; // down position

    Servo jewelKick;
    public static final double JEWEL_KICK_RIGHT = 0.8; // start (rest) position and counterclockwise kick
    public static final double JEWEL_KICK_LEFT = 0.15; // clockwise kick
    public static final double JEWEL_KICK_CENTER = 0.47;

    @Override
    public void init() {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry);
        mecanumWheels.powerMotors(0, 0, 0);
        lift = hardwareMap.dcMotor.get("Lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        touchSensor = hardwareMap.digitalChannel.get("Touch-Sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        touchSensorReleased = touchSensor.getState();
        if (!touchSensorReleased) {
            resetEncoders(lift, true);

        }
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(0);
        telemetry();
    }

    @Override
    public void start() {
        backButtonPressed = false;
        leftHand = hardwareMap.servo.get("Left-Hand");
        setUpServo(leftHand, LEFT_HAND_IN_POS, LEFT_HAND_OUT_POS);
        rightHand = hardwareMap.servo.get("Right-Hand");
        setUpServo(rightHand, RIGHT_HAND_IN_POS, RIGHT_HAND_OUT_POS);
        jewelArm = hardwareMap.servo.get("Jewel-Arm");
        jewelArm.setPosition(JEWEL_ARM_HOME);
        jewelKick = hardwareMap.servo.get("Jewel-Kick");
        jewelKick.setPosition(JEWEL_KICK_RIGHT);
    }

    @Override
    public void loop() {

        controlLift();
        controlGrip();
        telemetry();

        // DRIVING

        //Changing direction from forward to backward and backward to forward
        //Gamepad back button does not work with motorola 3G
        if (gamepad1.y) {
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
                forward = MAX_DRIVE_POWER;
            } else if (gamepad1.dpad_down) {
                forward = -MAX_DRIVE_POWER;
            }
            if (gamepad1.dpad_right) {
                right = MAX_DRIVE_POWER;
            } else if (gamepad1.dpad_left) {
                right = -MAX_DRIVE_POWER;
            }
            mecanumWheels.powerMotors(forward, right, clockwise);
        } else {
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            mecanumWheels.powerMotors(forward, right, clockwise);
        }
    }

    private void controlGrip() {
        float percentOpen = gamepad2.left_trigger;
        setPercentOpen(rightHand, percentOpen);
        setPercentOpen(leftHand, percentOpen);
    }

    private void controlLift() {

        // touchSensor.getState==true means the button is NOT PRESSED
        boolean touchPressed = !touchSensor.getState();
        if (touchPressed) {
            if (touchSensorReleased) {
                resetEncoders(lift, false);
                touchSensorReleased = false;
            } else {
                if (lift.getCurrentPosition() < 50 && !lift.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                    lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

        } else {
            touchSensorReleased = true;
        }

        if (gamepad2.y && lift.getCurrentPosition() < LIFT_COUNT_MAX) {
            lift.setPower(0.6);
        } else if (gamepad2.x && !touchPressed) {
            lift.setPower(-0.35);
        } else {
            lift.setPower(0);
        }
    }

    /**
     * Takes the holding position and release position,
     * Makes them the minimum and maximum servo positions
     * After this method, you can pass percentOpen as the servo position.
     * Holding position would be 0, complete release position would be 1.
     *
     * @param servo  - servo motor
     * @param inPos  - holding position between 0 and 1
     * @param outPos - releasing position between 0 and 1
     */
    public static void setUpServo(Servo servo, double inPos, double outPos) {
        // scale t0 the range between inPos and outPos
        double min = Math.min(inPos, outPos);
        double max = Math.max(inPos, outPos);
        servo.scaleRange(min, max);

        // make sure in position = 0, out position = 1
        if (inPos > outPos) {
            servo.setDirection(Servo.Direction.REVERSE);
        } else {
            servo.setDirection(Servo.Direction.FORWARD);
        }

        setPercentOpen(servo, 0);
    }

    /**
     * Sets percent open
     *
     * @param servo       - servo motor to use
     * @param percentOpen is the number between 0 and 1
     */
    public static void setPercentOpen(Servo servo, double percentOpen) {
        servo.setPosition(percentOpen);
    }

    void resetEncoders(DcMotor motor, boolean wait) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (wait) {
            while (motor.getCurrentPosition() > 10) {

                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    break;
                }
            }
        }

    }

    void telemetry() {
        telemetry.addData("lift", lift.getCurrentPosition());
        telemetry.addData("touch sensor state", touchSensor.getState());
        telemetry.addData("touch sensor released", touchSensorReleased);
    }
}