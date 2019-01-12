package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

@TeleOp(name = "DriverRover", group = "Main")
public class DriverRover extends OpMode {
    //TODO: EDIT LATER ON
    private static final double MINPOWER = 0.1;

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor liftMotor;
    //private DcMotor debrisCollection;
    private DcMotor intakeExtend; //extend - positive, retract - negative
    private DcMotor deliveryExtend; //extend - positive, retract - negative
    private DcMotor deliveryRotate; // raise - positive, lower - negative (with reverse)

    private ServoImplEx hookServo;
    private Servo markerServo;
    private Servo intakeGateServo;
    private Servo boxServo;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    private TouchSensor liftTouch;
    private TouchSensor intakeExtendTouch;
    private TouchSensor deliveryExtendTouch;
    private TouchSensor deliveryDownTouch;

    //for now, we don't support robot reversing the forward direction
    private final boolean forward = true;

    boolean hookReleased = true; //when true, the hook is released
    boolean hookButtonPressed = false; //when true, the hook button is pressed down

    //optimal lift height encoder counts
    public static final int LATCHING_POS = 8350;
    public static final int LATCHING_POS_LOW = 7900;
    static final int LIFT_POS_CLEAR = 5700;

    static final int MAX_INTAKE_ARM_POS = 5100;
    static final int RETRACTED_INTAKE_ARM_POS = 200;

    static final int MAX_DELIVERY_ROTATE_POS = 1544;
    static final int DELIVERY_ROTATE_UP_POS = 1200;
    static final int MAX_DELIVERY_EXTEND_POS = 1625;

    static final double POS_GATE_CLOSED = 0.525;
    static final double POS_GATE_OPEN = 0.85;

    static final double POS_BUCKET_PARKED = 0.2;
    static final double POS_BUCKET_UP = 0.72;
    static final double POS_BUCKET_DOWN = POS_BUCKET_PARKED;
    //static final double POS_BUCKET_DROP = 0.2;


    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("wheelsLeft");
        motorRight = hardwareMap.dcMotor.get("wheelsRight");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        intakeExtend = hardwareMap.dcMotor.get("intakeExtend");
        deliveryExtend = hardwareMap.dcMotor.get("deliveryExtend");
        deliveryRotate = hardwareMap.dcMotor.get("deliveryRotate");

        // run by speed
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // touch sensors
        liftTouch = hardwareMap.touchSensor.get("lift_touch");
        intakeExtendTouch = hardwareMap.touchSensor.get("intakeExtendTouch");
        deliveryDownTouch = hardwareMap.touchSensor.get("deliveryDownTouch");
        deliveryExtendTouch = hardwareMap.touchSensor.get("deliveryExtendTouch");

        // float zero power
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorRight.setDirection(DcMotor.Direction.REVERSE);


        deliveryRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        deliveryRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (liftTouch.isPressed()) {
            //reset the encoder for the lift motor
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (deliveryDownTouch.isPressed()) {
            // reset encoders for the rotating delivery arm
            deliveryRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (intakeExtendTouch.isPressed()) {
            intakeExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        sleep(100);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        deliveryRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        log();
    }

    @Override
    public void start() {

        //ServoImplEx allows to energize and deenergize servo
        // we don't want to hook servo to keep position when robot is lifting or lowering
        hookServo = (ServoImplEx) hardwareMap.servo.get("hookServo");
        //set position to 0 for releasing the hook and use position 1 to close hook
        setUpServo(hookServo, AutonomousRover.POS_HOOK_OPEN, AutonomousRover.POS_HOOK_CLOSED);
        deenergizeHook();

        markerServo = hardwareMap.servo.get("markerServo");
        //for moving the marker hand forward, use 1, for moving it back, use 0
        setUpServo(markerServo, AutonomousRover.POS_MARKER_BACK, AutonomousRover.POS_MARKER_FORWARD);


        intakeGateServo = hardwareMap.servo.get("intakeGate");
        //for opening and closing gate that drops minerals into delivery, use 1 and 0
        setUpServo(intakeGateServo, POS_GATE_CLOSED, POS_GATE_OPEN);

        boxServo = hardwareMap.servo.get("box");
        //for bucket servo that delivers the minerals to the launcher
        setUpServo(boxServo, POS_BUCKET_PARKED, POS_BUCKET_UP);

        rightIntakeServo = hardwareMap.crservo.get("rightIntake");
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo = hardwareMap.crservo.get("leftIntake");

    }

    @Override
    public void loop() {

        //gamepad2 will be used to control the delivery of the minerals

        if (gamepad2.left_trigger > 0.5) {
            setPercentOpen(intakeGateServo, 1);
        } else {
            setPercentOpen(intakeGateServo, 0);
        }

        //if the delivery arm is down, the bucket should be in bottom parked position.
        // 0 corresponds to the parked position (or drop) 1 - to vertical position,
        //    when the delivery arm is up
        if (deliveryDownTouch.isPressed()) {
            boxServo.setPosition(0);
        } else {
            if (deliveryRotate.getCurrentPosition() < DELIVERY_ROTATE_UP_POS) {
                boxServo.setPosition(((double) deliveryRotate.getCurrentPosition()) / DELIVERY_ROTATE_UP_POS);
            }

            //when the arm is up, we want the be able to invoke trigger to drop minerals.
            if (deliveryRotate.getCurrentPosition() > DELIVERY_ROTATE_UP_POS) {
                if (gamepad2.right_trigger > 0.5) {
                    boxServo.setPosition(0);
                } else {
                    boxServo.setPosition(1);
                }
            }
        }

        if (gamepad2.y && deliveryExtend.getCurrentPosition() < MAX_DELIVERY_EXTEND_POS) {
            deliveryExtend.setPower(0.7);
        } else if (gamepad2.a && !deliveryExtendTouch.isPressed()) {
            deliveryExtend.setPower(-0.7);
        } else {
            deliveryExtend.setPower(0);
        }

        if (gamepad2.b && deliveryRotate.getCurrentPosition() < MAX_DELIVERY_ROTATE_POS) {
            deliveryRotate.setPower(0.5);
        } else if (gamepad2.x && !deliveryDownTouch.isPressed()) {
            if (deliveryRotate.getCurrentPosition() < DELIVERY_ROTATE_UP_POS) {
                deliveryRotate.setPower(-deliveryRotate.getCurrentPosition() / (DELIVERY_ROTATE_UP_POS * 10.0));
                if (deliveryRotate.getPower() > -0.06) {
                    deliveryRotate.setPower(-0.06);
                }
            } else {
                deliveryRotate.setPower(-0.3);
            }
        } else {
            deliveryRotate.setPower(0);
        }


        if (gamepad1.y)

        {
            rightIntakeServo.setPower(1);
            leftIntakeServo.setPower(1);
        } else if (gamepad1.b)

        {
            rightIntakeServo.setPower(-1);
            leftIntakeServo.setPower(-1);
        } else

        {
            leftIntakeServo.setPower(0.0);
            rightIntakeServo.setPower(0.0);
        }

        //power given to the left side wheels, power given to the right side wheels
        //forward as a direction
        double leftForward;
        double rightForward;
        double liftMotorPower;
        double intakeExtendPower;

        int sign = forward ? 1 : -1;
        //Arcade Drive
        rightForward = -(

                scaled(gamepad1.left_stick_y) + sign *

                        scaled(gamepad1.left_stick_x));
        leftForward = -(

                scaled(gamepad1.left_stick_y) - sign *

                        scaled(gamepad1.left_stick_x));

        //Tank Drive
        //leftForward = -scaled(gamepad1.left_stick_y);
        //rightForward = -scaled(gamepad1.right_stick_y);


        if ((gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right))

        {
            // dPadDrive: Forward, Backwards, in place CClockwise and Clockwise
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

        // Lift motor when given negative power, the lift rises and lowers when given positive power
        if (gamepad1.x || gamepad1.a)

        {
            int currentPos = Math.abs(liftMotor.getCurrentPosition());
            if (gamepad1.x) {
                if (LATCHING_POS < currentPos) {
                    liftMotorPower = 0;
                } else {
                    liftMotorPower = -1;
                }
            } else if (!liftTouch.isPressed()) { //a button is pressed
//                if (currentPos < 300) {
//                    liftMotorPower = 0.7;
//                } else {
                if (currentPos < LATCHING_POS_LOW) {
                    deenergizeHook();
                }
                liftMotorPower = 1;
//                }
            } else {
                liftMotorPower = 0.00;
            }
        } else

        {
            liftMotorPower = 0.0;
        }

        if (gamepad1.b)

        {
            hookButtonPressed = true;
        } else

        {
            if (hookButtonPressed) {
                energizeHook();
                if (hookReleased) {
                    hookServo.setPosition(1);
                } else {
                    hookServo.setPosition(0);
                }
                hookReleased = !hookReleased;
            }
            hookButtonPressed = false;
        }

        // Lift motor when given negative power, the lift rises and lowers when given positive power
        if (gamepad1.left_bumper || gamepad1.right_bumper)

        {
            if (gamepad1.left_bumper && intakeExtend.getCurrentPosition() < MAX_INTAKE_ARM_POS) {
                intakeExtendPower = -0.5; // going out
            } else if (gamepad1.right_bumper && !intakeExtendTouch.isPressed()) {
                intakeExtendPower = 0.5; // going in
            } else {
                //touch button is pressed - cannot go further
                intakeExtendPower = 0.0;
            }
        } else

        {
            intakeExtendPower = 0.0;
        }

        //driving backwards
        if (!forward)

        { //when start, front direction is the intake side, lightStrip2
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

        powerMotors(rightForward, leftForward, liftMotorPower, intakeExtendPower);

        log();
        telemetry.addData("drive power left / right",
                leftForward + "/" + rightForward);
    }

    private void log() {
        telemetry.addData("intake extend / touch",
                intakeExtend.getCurrentPosition() + "/" + intakeExtendTouch.isPressed());
        telemetry.addData("delivery rotate / touch",
                deliveryRotate.getCurrentPosition() + "/" + deliveryDownTouch.isPressed());
        telemetry.addData("delivery extend / touch",
                deliveryExtend.getCurrentPosition() + "/" + deliveryExtendTouch.isPressed());
        telemetry.addData("lift / touch",
                liftMotor.getCurrentPosition() + "/" + liftTouch.isPressed());

    }

    private void powerMotors(double rightForward, double leftForward, double liftMotorPower, double intakeExtendPower) {
        motorLeft.setPower(leftForward);
        motorRight.setPower(rightForward);
        liftMotor.setPower(liftMotorPower);
        intakeExtend.setPower(intakeExtendPower);
    }

    /**
     * deenergize hook servo (if enabled)
     */
    void deenergizeHook() {
        if (hookServo.isPwmEnabled()) {
            hookServo.setPwmDisable();
        }
    }

    /**
     * energize hook servo (if disabled)
     */
    void energizeHook() {
        if (!hookServo.isPwmEnabled()) {
            hookServo.setPwmEnable();
        }
    }


    private double scaled(double x) {
        return (x / 1.07) * (.62 * x * x + .45);
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
    static void setUpServo(Servo servo, double inPos, double outPos) {
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
    static void setPercentOpen(Servo servo, double percentOpen) {
        servo.setPosition(percentOpen);
    }

}