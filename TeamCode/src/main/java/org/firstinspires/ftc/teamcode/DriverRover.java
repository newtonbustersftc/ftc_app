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
    private DcMotor intakeExtend; //extend - positive, retract - negative
    private DcMotor deliveryExtend; //extend - positive, retract - negative
    private DcMotor deliveryRotate; // raise - positive, lower - negative (with reverse)

    private ServoImplEx hookServo;
    private Servo markerServo;
    private Servo intakeGateServo;
    private Servo boxServo;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private Servo intakeHolder;

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

    /**
     * The encoder counts for intakeExtend motor are going from 0 to negative on extension
     * When changing the motor also check the logic in IsIntakeMaxExtended/MinRetracted
     */
    static final int MAX_INTAKE_ARM_POS = -5200;
    static final int ALMOST_MAX_INTAKE_ARM_POS = -4700;
    static final int ALMOST_MIN_INTAKE_ARM_POS = -250;

    static final int DELIVERY_ROTATE_MAX_POS = 1250;
    static final int DELIVERY_ROTATE_UP_POS = 1050;
    static final int DELIVERY_ROTATE_BEFORE_HOME_POS = 400;

    static int MAX_DELIVERY_EXTEND_POS = 1550;
    static int ALMOST_MAX_DELIVERY_EXTEND_POS = MAX_DELIVERY_EXTEND_POS - 400;
    static final int ALMOST_MIN_DELIVERY_EXTEND_POS = 400;

    static final double POS_GATE_CLOSED = 0.525;
    static final double POS_GATE_OPEN = 0.85;

    static final double POS_INTAKE_HOLD = 0.421;
    static final double POS_INTAKE_RELEASE = 0.65;
    static final double POS_INTAKE_RELEASE_EXTREME = 0.7;

    static final double POS_BUCKET_PARKED = 0.2115;
    static final double POS_BUCKET_UP = 0.6825;
    static final double POS_BUCKET_DROP = POS_BUCKET_PARKED;

    boolean switchExtention = false;
    boolean fullExtention = true;

    enum DeliveryState {
        HOME, BEFORE_HOME, ARM_UP, DELIVERY
    }

    DeliveryState deliveryArmState;
    boolean intakeReleased;
    private static boolean isError; //IS there an error?

    // power is positive for forward direction, negative for backward direction
    double leftForward; //power given to the left side wheels
    double rightForward; //power given to the right side wheels


    static final double HALF_WIDTH = 7.5; //width between wheels is 15 inches

    static final double ARC_MIN_RADIUS = 10.0;
    static final double ARC_MAX_RADIUS = 40.0;

    @Override
    public void init() {
        Lights.setUpLights(hardwareMap);
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
        Lights.blue(deliveryDownTouch.isPressed());


        // float zero power
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        deliveryExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        deliveryRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        deliveryRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        if (liftTouch.isPressed()) {
            //reset the encoder for the lift motor
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (deliveryDownTouch.isPressed()) {
            // reset encoders for the rotating delivery arm
            deliveryRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (deliveryExtendTouch.isPressed()) {
            // reset encoders for the rotating delivery arm
            deliveryExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (intakeExtendTouch.isPressed()) {
            intakeExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        sleep(100);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        deliveryRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        deliveryExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // if the driver mode is restarted the arm might not be in HOME state
        restoreDeliveryArmState();
        Lights.green(true);

        if (!gamepad1.atRest() || !gamepad2.atRest()) {
            Lights.red(true);
        }

        log();
    }

    private void restoreDeliveryArmState() {
        int currentPos = deliveryRotate.getCurrentPosition();
        if (currentPos > DELIVERY_ROTATE_UP_POS) {
            deliveryArmState = DeliveryState.DELIVERY;
        } else if (currentPos > DELIVERY_ROTATE_BEFORE_HOME_POS) {
            deliveryArmState = DeliveryState.ARM_UP;
        } else if (!deliveryDownTouch.isPressed()) {
            deliveryArmState = DeliveryState.BEFORE_HOME;
        } else {
            deliveryArmState = DeliveryState.HOME;
        }
    }

    @Override
    public void start() {
        Lights.resetLights();
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

        intakeHolder = hardwareMap.servo.get("intakeHolder");
        //intakeHolder.setPosition(POS_INTAKE_HOLD);
        intakeReleased = isIntakeReleased();

        boxServo = hardwareMap.servo.get("box");
        //for bucket servo that delivers the minerals to the launcher
        setUpServo(boxServo, POS_BUCKET_PARKED, POS_BUCKET_UP);

        rightIntakeServo = hardwareMap.crservo.get("rightIntake");
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo = hardwareMap.crservo.get("leftIntake");
    }

    boolean isIntakeReleased() {
        double pos = intakeHolder.getPosition();
        return pos > POS_INTAKE_RELEASE - 0.01;
    }

    @Override
    public void loop() {
        isError = false;

        //gamepad2 will be used to control the delivery of the minerals
        if (gamepad2.y) {
            switchExtention = true;
        } else {
            if (switchExtention) {
                if (fullExtention) {
                    MAX_DELIVERY_EXTEND_POS = MAX_DELIVERY_EXTEND_POS / 2;
                    ALMOST_MAX_DELIVERY_EXTEND_POS = ALMOST_MAX_DELIVERY_EXTEND_POS / 2;
                } else {
                    MAX_DELIVERY_EXTEND_POS = MAX_DELIVERY_EXTEND_POS * 2;
                    ALMOST_MAX_DELIVERY_EXTEND_POS = ALMOST_MAX_DELIVERY_EXTEND_POS * 2;
                }
                fullExtention = !fullExtention;
                switchExtention = false;
            }
        }

        int deliveryRotatePos = deliveryRotate.getCurrentPosition();
        if (gamepad2.left_trigger > 0.5 ||
                (isIntakeMinRetracted() && isDeliveryArmDown(deliveryRotatePos))) {
            setPercentOpen(intakeGateServo, 1);
        } else {
            setPercentOpen(intakeGateServo, 0);
        }

        //if the delivery arm is down, the bucket should be in bottom parked position.
        // 0 corresponds to the parked position (or drop) 1 - to vertical position,
        //    when the delivery arm is up
        if (deliveryDownTouch.isPressed() || deliveryRotatePos < 100) {
            boxServo.setPosition(0);
        } else {
            if (deliveryRotatePos < DELIVERY_ROTATE_UP_POS) {
                boxServo.setPosition(((double) deliveryRotatePos) / DELIVERY_ROTATE_UP_POS);
            }

            //when the arm is up, we want the be able to invoke trigger to drop minerals.
            if (deliveryRotatePos > DELIVERY_ROTATE_UP_POS) {
                if (gamepad2.right_trigger > 0.2) {
                    /*
                     * When dropping debris, servo position 1 when
                     * box is vertical and position 0 when box is down.
                     */
                    boxServo.setPosition(1 - gamepad2.right_trigger); //Uses analog trigger position
                } else {
                    boxServo.setPosition(1);
                }
            }
        }

        // if using automatic arm control, shouldn't allow for manual control for arm extension
        if (gamepad2.left_stick_y < -0.5) {
            toDeliveryPosition();
        } else if (gamepad2.left_stick_y > 0.5) {
            toHomePosition();
        } else {
            if (!deliveryArmState.equals(DeliveryState.DELIVERY)) deliveryRotate.setPower(0);

            //manual control for arm extension (right stick) only used when not using auto (left stick)
            if (gamepad2.right_stick_y < -0.2) {
                extendDeliveryArm(-gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y > 0.2) {
                retractDeliveryArm(-gamepad2.right_stick_y);
            } else {
                deliveryExtend.setPower(0);
            }
        }

        if (gamepad1.y) {
            rightIntakeServo.setPower(1);
            leftIntakeServo.setPower(1);
        } else if (gamepad1.b) {
            rightIntakeServo.setPower(-1);
            leftIntakeServo.setPower(-1);
        } else {
            leftIntakeServo.setPower(0.0);
            rightIntakeServo.setPower(0.0);
        }


        if (gamepad1.b) {
            hookButtonPressed = true;
        } else {
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

        //driver control to release the intake holder (hook)
        if (gamepad1.left_bumper) {
            intakeHolder.setPosition(POS_INTAKE_RELEASE_EXTREME);
            intakeReleased = true;
        }

        if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            doArc ();
        }
        else {
            controlWheels();
        }

        controlIntake();
        controlLift();

        log();

        // If error, then RED LIGHT
        Lights.red(isError);
        // If not fully extended (halfway), then YELLOW LIGHT
        Lights.yellow(!fullExtention);
        // If delivery arm is all the way down, then BLUE LIGHT
        Lights.blue(isDeliveryArmDown(deliveryRotatePos));

    }

    private void toHomePosition() {
        if (deliveryDownTouch.isPressed()) {
            deliveryRotate.setPower(0);
            deliveryArmState = DeliveryState.HOME;
            return;
        }

        int currentPos = deliveryRotate.getCurrentPosition();
        switch (deliveryArmState) {
            case HOME:
                if (!deliveryDownTouch.isPressed()) {
                    deliveryRotate.setPower(0.16); //Was 0.08
                } else {
                    deliveryRotate.setPower(0);
                }
                break;
            case BEFORE_HOME:
                retractDeliveryArm(-1);
                if (currentPos < DELIVERY_ROTATE_BEFORE_HOME_POS) {
                    deliveryRotate.setPower(0);
                    if (deliveryExtendTouch.isPressed()) {
                        deliveryRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        deliveryRotate.setTargetPosition(0);
                        deliveryArmState = DeliveryState.HOME;
                    }
                } else {
                    deliveryRotate.setPower(-0.2);
                }
                break;
            case ARM_UP:
                deliveryRotate.setPower(-0.2);
                if (currentPos < DELIVERY_ROTATE_UP_POS) {
                    deliveryArmState = DeliveryState.BEFORE_HOME;
                }
                break;
            case DELIVERY:
                deliveryRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                deliveryRotate.setPower(-0.5);
                deliveryArmState = DeliveryState.ARM_UP;
                break;
        }
    }


    private void toDeliveryPosition() {
        int currentPos = deliveryRotate.getCurrentPosition();

        switch (deliveryArmState) {
            case HOME:
            case BEFORE_HOME:
                deliveryRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                deliveryRotate.setPower(0.6);
                deliveryArmState = DeliveryState.ARM_UP;
                break;
            case ARM_UP:
                if (currentPos > DELIVERY_ROTATE_UP_POS) {
                    deliveryRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    deliveryRotate.setTargetPosition(DELIVERY_ROTATE_MAX_POS);
                    deliveryRotate.setPower(0.8);
                    deliveryArmState = DeliveryState.DELIVERY;
                } else {
                    deliveryRotate.setPower(0.6);
                    if (currentPos > DELIVERY_ROTATE_BEFORE_HOME_POS) {
                        extendDeliveryArm(1);
                    }
                }
                break;
            case DELIVERY:
                extendDeliveryArm(1);
                break;
        }
    }

    private void extendDeliveryArm(double power) {
        int currCounts = deliveryExtend.getCurrentPosition();

        if (currCounts > MAX_DELIVERY_EXTEND_POS) {
            deliveryExtend.setPower(0);
        } else if (currCounts > ALMOST_MAX_DELIVERY_EXTEND_POS) {
            deliveryExtend.setPower(0.2);
        } else {
            deliveryExtend.setPower(Math.abs(power));
        }
    }

    private void retractDeliveryArm(double power) {
        if (deliveryExtendTouch.isPressed()) {
            deliveryExtend.setPower(0);
        } else if (deliveryExtend.getCurrentPosition() < ALMOST_MIN_DELIVERY_EXTEND_POS) {
            deliveryExtend.setPower(-0.2);
        } else {
            deliveryExtend.setPower(-Math.abs(power));
        }
    }


    private void log() {
        telemetry.addData("drive power left / right",
          leftForward + "/" + rightForward);
        telemetry.addData("intake extend / touch",
                intakeExtend.getCurrentPosition() + "/" + intakeExtendTouch.isPressed());
        telemetry.addData("delivery rotate / touch / state",
                deliveryRotate.getCurrentPosition() + "/" +
                        deliveryDownTouch.isPressed() + "/" + deliveryArmState);
        telemetry.addData("delivery extend / touch",
                deliveryExtend.getCurrentPosition() + "/" + deliveryExtendTouch.isPressed());
        telemetry.addData("lift / touch",
                liftMotor.getCurrentPosition() + "/" + liftTouch.isPressed());
        if (intakeHolder != null) {
            telemetry.addData("Holder Pos", intakeHolder.getPosition());
        }
    }

    //assuming specialist dpad is controlling robot movements
    private void doArc() {
        double POWER = 0.5;
        //backward moving from our crater to depot side launcher
        //forward moving from depot side launcher to our crater
        int sign = gamepad1.right_stick_x < 0 ? 1 : -1;

        double radius = ARC_MAX_RADIUS - Math.abs(gamepad1.right_stick_x) * (ARC_MAX_RADIUS - ARC_MIN_RADIUS);
        double powerRatio = (radius - HALF_WIDTH) / (radius + HALF_WIDTH);
        rightForward = POWER;
        leftForward = POWER / powerRatio;
        if(leftForward > 1) {
            rightForward = rightForward/leftForward;
            leftForward = leftForward/leftForward;
        }
        //outer motor is always left
        motorLeft.setPower (sign * leftForward);
        motorRight.setPower (sign * rightForward);

    }

    private void controlWheels() {

        int sign = forward ? 1 : -1;
        //Arcade Drive
        rightForward = -(scaled(gamepad1.left_stick_y) + sign * scaled(gamepad1.left_stick_x));
        leftForward = -(scaled(gamepad1.left_stick_y) - sign * scaled(gamepad1.left_stick_x));

        //Tank Drive
        //leftForward = -scaled(gamepad1.left_stick_y);
        //rightForward = -scaled(gamepad1.right_stick_y);


        if ((gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right)) {
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
        //driving backwards
//        if (!forward) { //when start, front direction is the intake side, lightStrip2
//            leftForward = -leftForward;
//            rightForward = -rightForward;
//        }

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


        motorLeft.setPower(leftForward);
        motorRight.setPower(rightForward);
    }


    private void controlLift() {

        double liftMotorPower;
        // Lift motor when given negative power, the lift rises and lowers when given positive power
        if (gamepad1.x || gamepad1.a) {
            int currentPos = Math.abs(liftMotor.getCurrentPosition());
            if (gamepad1.x) {
                if (LATCHING_POS < currentPos) {
                    liftMotorPower = 0;
                } else {
                    liftMotorPower = -1;
                }
            } else if (!liftTouch.isPressed()) {
                if (currentPos < LATCHING_POS_LOW) {
                    deenergizeHook();
                }
                liftMotorPower = 1;
            } else {
                liftMotorPower = 0.00;
            }
        } else {
            liftMotorPower = 0.0;
        }
        liftMotor.setPower(liftMotorPower);
    }

    private void controlIntake() {

        double intakeExtendPower;
        double mintriggervalue = 0.3;
        if (gamepad1.left_trigger > mintriggervalue || gamepad1.right_trigger > mintriggervalue) {
            if (gamepad1.left_trigger > mintriggervalue && !isIntakeMaxExtended()) {
                // going out
                if (!intakeReleased) {
                    isError = true;
                    intakeExtendPower = 0;
                } else if (isIntakeAlmostMaxExtended()) {
                    intakeExtendPower = -mintriggervalue;
                } else {
                    intakeExtendPower = -gamepad1.left_trigger;
                }
            } else if (gamepad1.right_trigger > mintriggervalue && !intakeExtendTouch.isPressed()) {
                // retracting intake arm
                if (isIntakeAlmostMinRetracted()) {
                    intakeExtendPower = mintriggervalue;
                } else {
                    intakeExtendPower = gamepad1.right_trigger;
                }
            } else {
                //touch button is pressed - cannot go further
                intakeExtendPower = 0.0;
            }
        } else {
            intakeExtendPower = 0.0;
        }
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

    private boolean isDeliveryArmDown(int deliveryRotatePos) {
        return deliveryDownTouch.isPressed() || deliveryRotatePos < 20;
    }

    private boolean isIntakeMaxExtended() {
        return intakeExtend.getCurrentPosition() < MAX_INTAKE_ARM_POS;
    }

    private boolean isIntakeAlmostMaxExtended() {
        return intakeExtend.getCurrentPosition() < ALMOST_MAX_INTAKE_ARM_POS;

    }

    private boolean isIntakeMinRetracted() {
        return intakeExtendTouch.isPressed();
    }

    private boolean isIntakeAlmostMinRetracted() {
        return intakeExtend.getCurrentPosition() > ALMOST_MIN_INTAKE_ARM_POS;

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