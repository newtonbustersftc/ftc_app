package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

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
import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.teamcode.AutonomousOptionsRover.AUTO_MODE_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptionsRover.getSharedPrefs;

@TeleOp(name = "DriverRover", group = "Main")
public class DriverRover extends OpMode {
    //TODO: EDIT LATER ON
    private static final double MINPOWER = 0.1;

    // encoder positions for lift motor
    static final int LATCHING_POS = 8350; // middle of the handle
    static final int LATCHING_POS_LOW = 7900; // lowest part of the handle
    static final int LIFT_POS_CLEAR = 5700; // below the handle

    // encoder positions for the intake arm motor
    // 0 - arm is completely retracted
    // IMPORTANT: when changing the motor, also check logic in isIntake*Extended/*Retracted
    static final int MAX_INTAKE_ARM_POS = -5200;
    static final int ALMOST_MAX_INTAKE_ARM_POS = -4700; // TODO: CHECK - can it be increased?
    static final int ALMOST_MIN_INTAKE_ARM_POS = -250;

    // encoder positions for the rotating delivery arm motor
    // 0 - arm all the way down
    static final int DELIVERY_ROTATE_MAX_POS = 1250;
    static final int DELIVERY_ROTATE_VERT_POS = 1190; // arm is vertical
    static final int DELIVERY_ROTATE_UP_POS = 1050;
    static final int DELIVERY_ROTATE_BEFORE_HOME_POS = 400;
    static final int DELIVERY_ROTATE_HORIZ_POS = 350; // arm is horizontal

    // encode positions for the delivery extension motor
    // 0 - arm is not extended
    static int MAX_DELIVERY_EXTEND_POS = 1550;
    static int ALMOST_MAX_DELIVERY_EXTEND_POS = MAX_DELIVERY_EXTEND_POS - 200;
    static final int ALMOST_MIN_DELIVERY_EXTEND_POS = 200;

    static final double POS_GATE_CLOSED = 0.525;
    static final double POS_GATE_OPEN = 0.85;

    static final double POS_INTAKE_HOLD = 0.421;
    static final double POS_INTAKE_RELEASE = 0.65;
    static final double POS_INTAKE_RELEASE_EXTREME = 0.7;

    static final double POS_BUCKET_PARKED = 0.2115; // also used as drop position
    static final double POS_BUCKET_UP = 0.6825;

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

    private boolean hookReleased = true; //when true, the hook is released
    private boolean hookButtonPressed = false; //when true, the hook button is pressed down

    private boolean switchExtension = false;
    private static boolean fullExtension = true;

    private boolean switchCrater = false;
    private boolean ourCrater = false;

    private static final long BLINK_START_MILLIS = 95000;
    private static final int INIT_BLINK_WAIT = 1800;

    enum BlinkerState {
        NONE, ON , OFF , SUCCESS
    }

    private BlinkerState blinkerState = BlinkerState.NONE;
    private long startTime;
    private long blinkerTime;
    private int blinkerWait = INIT_BLINK_WAIT;
    private int liftStartPos;

    enum DeliveryState {
        HOME, BEFORE_HOME, ARM_UP, DELIVERY
    }

    private DeliveryState deliveryArmState;
    private boolean intakeReleased;
    private static boolean isError; //IS there an error?

    // power is positive for forward direction, negative for backward direction
    private double leftForward; //power given to the left side wheels
    private double rightForward; //power given to the right side wheels


    private static final double HALF_WIDTH = 7.5; //width between wheels is 15 inches

    // minimum and maximum radius of arc move
    // between crater and depot side launcher
    private static final double ARC_MIN_RADIUS = 20.0;
    private static final double ARC_MAX_RADIUS = 35.0;

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

        //save Intitial lift pos for blinker
        liftStartPos = liftMotor.getCurrentPosition();

        // is we've ran autonomous from depot side, we are starting driver mode
        // at the opposite alliance crater and the first delivery will be to the depot side
        SharedPreferences prefs = getSharedPrefs(hardwareMap);
        String autoMode = prefs.getString(AUTO_MODE_PREF, "");
        ourCrater = autoMode.equals(AutonomousOptionsRover.AutoMode.CRATER.toString());

        if (!ourCrater) {
            switchExtension();
        }

        Lights.green(true);
        Lights.yellow(!fullExtension);
        Lights.white(!ourCrater);
        Lights.blue(deliveryDownTouch.isPressed());

        if (!gamepad1.atRest() || !gamepad2.atRest()) {
            Lights.red(true);
        }

        log();
    }

    @Override
    public void start() {
        startTime = currentTimeMillis();
        blinkerTime = currentTimeMillis();

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

    @Override
    public void loop() {
        isError = false;

        // switch crater
        // the forward arc is clockwise to our crater,
        // counterclockwise to opposite crater
        if (gamepad1.right_bumper) {
            if (!switchCrater) {
                ourCrater = !ourCrater;
                switchCrater = true;
            }
        } else {
            switchCrater = false;
        }

        if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            doArc();
        }
        else {
            controlWheels();
        }

        int deliveryRotatePos = controlDelivery();
        controlIntake();
        controlLift();
        controlBlinker();

        if (blinkerState != BlinkerState.SUCCESS) {
            // If opposite crater WHITE LIGHT
            // used for arc moves
            Lights.white(!ourCrater);
            // If not fully extended (halfway), then YELLOW LIGHT
            Lights.yellow(!fullExtension);
            // If delivery arm is all the way down, then BLUE LIGHT
            Lights.blue(isDeliveryArmDown(deliveryRotatePos));
        }

        log();
    }

    private void controlBlinker() {
        switch (blinkerState) {
            //Start blinking 25 sec before the end
            case NONE:
                if (currentTimeMillis() - startTime > BLINK_START_MILLIS) {
                    blinkerState = BlinkerState.ON;
                    Lights.green(true);
                    Lights.red(true);
                    blinkerTime = currentTimeMillis();
                } else {
                    // If error, then RED LIGHT
                    Lights.red(isError);
                }
                break;
            case ON:
                if(currentTimeMillis() - blinkerTime > blinkerWait){
                    blinkerState = BlinkerState.OFF;

                    //blinkerWait depends on how close you are to the end of the game (time-wise)
                    long millisFromStart = currentTimeMillis() - startTime;

                    if (millisFromStart < BLINK_START_MILLIS + 10000) {
                        blinkerWait = INIT_BLINK_WAIT;
                    } else if (millisFromStart < BLINK_START_MILLIS + 15000) {
                        blinkerWait = INIT_BLINK_WAIT / 2;
                    } else {
                        blinkerWait = INIT_BLINK_WAIT / 4;
                    }

                    Lights.green(false);
                    Lights.red(false);
                    blinkerTime = currentTimeMillis();
                }
                break;
            case OFF:
                if(Math.abs(liftMotor.getCurrentPosition() - liftStartPos) > LIFT_POS_CLEAR/1.5 ){
                    blinkerState = BlinkerState.SUCCESS;
                    Lights.resetLights();
                    Lights.green(true);
                } else if(currentTimeMillis() - blinkerTime > blinkerWait) {
                    blinkerState = BlinkerState.ON;
                    Lights.green(true);
                    Lights.red(true);
                    blinkerTime = currentTimeMillis();
                }
                break;
            case SUCCESS: //just a placeholder
                break;
        }
    }

    private int controlDelivery() {
        //gamepad2 will be used to control the delivery of the minerals
        if (gamepad2.y) {
            switchExtension = true;
        } else {
            if (switchExtension) {
                switchExtension();
                switchExtension = false;
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
        return deliveryRotatePos;
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
                        if (fullExtension) extendDeliveryArm(1);
                    }
                }
                break;
            case DELIVERY:
                if (fullExtension) extendDeliveryArm(1);
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

    /**
     * Switches delivery arm maximum extension to the optimal length for depot side delivery
     */
    private void switchExtension() {
//        if (fullExtension) {
//            MAX_DELIVERY_EXTEND_POS = MAX_DELIVERY_EXTEND_POS / 2;
//            ALMOST_MAX_DELIVERY_EXTEND_POS = ALMOST_MAX_DELIVERY_EXTEND_POS / 2;
//        } else {
//            MAX_DELIVERY_EXTEND_POS = MAX_DELIVERY_EXTEND_POS * 2;
//            ALMOST_MAX_DELIVERY_EXTEND_POS = ALMOST_MAX_DELIVERY_EXTEND_POS * 2;
//        }
        fullExtension = !fullExtension;
    }

    /**
     * Arc moves between our or opposite alliance crater and depot side launcher,
     * driver's right stick x is proportional to the radius, bigger values - sharper turn
     * When moving from/to opposite alliance crater right wheel is outer (further from the center of the arc)
     * When moving from/to our crater left wheel is outer
     */
    private void doArc() {
        double POWER = 0.5; // inner wheel power before scaling

        double radius = ARC_MAX_RADIUS - Math.abs(gamepad1.right_stick_x) * (ARC_MAX_RADIUS - ARC_MIN_RADIUS);
        double powerRatio = (radius - HALF_WIDTH) / (radius + HALF_WIDTH);
        double innerForward = POWER;
        double outerForward = POWER / powerRatio;
        if(outerForward > 1) {
            innerForward = innerForward/outerForward;
            outerForward = outerForward/outerForward;
        }
        // outer motor is left between our crater and our depot side launcher
        // outer motor is right between opposite crater and our depot side launcher
        if (ourCrater) {
            //backward moving from our crater to depot side launcher
            //forward moving from depot side launcher to our crater
            int sign = gamepad1.right_stick_x < 0 ? 1 : -1;
            rightForward = sign * innerForward;
            leftForward = sign * outerForward;
        } else {
            int sign = gamepad1.right_stick_x < 0 ? -1 : 1;
            rightForward = sign * outerForward;
            leftForward = sign * innerForward;
        }
        motorLeft.setPower (leftForward);
        motorRight.setPower (rightForward);

    }

    private void controlWheels() {

        int sign = forward ? 1 : -1; // controls direction of turns

        if ((gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right)) {
            // dPadDrive: Forward, Backwards, in place CClockwise and Clockwise
            // We are using robot coordinates
            double dpadSpeed = 0.2;
            if (gamepad1.dpad_up) {
                rightForward = dpadSpeed*1.5;
                leftForward = dpadSpeed*1.5;
            } else if (gamepad1.dpad_down) {
                rightForward = -dpadSpeed*1.5;
                leftForward = -dpadSpeed*1.5;
            } else if (gamepad1.dpad_left) {
                //rotating ccw
                rightForward = sign * dpadSpeed;
                leftForward = -sign * dpadSpeed;
            } else {
                //rotating cw
                leftForward = sign * dpadSpeed;
                rightForward = -sign * dpadSpeed;
            }
        } else {

            //Arcade Drive
            rightForward = -(scaled(gamepad1.left_stick_y) + sign * scaled(gamepad1.left_stick_x));
            leftForward = -(scaled(gamepad1.left_stick_y) - sign * scaled(gamepad1.left_stick_x));

            //Tank Drive - not for this game: right stick is arc control now
            //leftForward = -scaled(gamepad1.left_stick_y);
            //rightForward = -scaled(gamepad1.right_stick_y);


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
        }

        // driving backwards
        if (!forward) {
            leftForward = -leftForward;
            rightForward = -rightForward;
        }

        motorLeft.setPower(leftForward);
        motorRight.setPower(rightForward);
    }


    private void controlLift() {

        // change the state of hook between open and closed
        // note: b is also used for reverse intake
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

        double liftMotorPower;
        // the lift arm rises when given negative power and lowers when given positive power
        if (gamepad1.x || gamepad1.a) {
            int currentPos = Math.abs(liftMotor.getCurrentPosition());
            if (gamepad1.x) {
                if (LATCHING_POS < currentPos) {
                    liftMotorPower = 0; // stop at mid handle
                } else {
                    liftMotorPower = -1;
                }
            } else if (!liftTouch.isPressed()) {
                if (currentPos < LATCHING_POS_LOW) {
                    deenergizeHook(); // deenergize hook when robot is hanging
                }
                liftMotorPower = 1;
            } else {
                liftMotorPower = 0;
            }
        } else {
            liftMotorPower = 0.0;
        }
        liftMotor.setPower(liftMotorPower);
    }

    private void controlIntake() {

        //driver control to release the intake holder (hook)
        //we should have released the hook at the end of autonomous,
        //but something could have gone wrong
        if (gamepad1.left_bumper) {
            intakeHolder.setPosition(POS_INTAKE_RELEASE_EXTREME);
            intakeReleased = true;
        }

        // intake wheel forward and reverse
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

        double intakeExtendPower;
        double mintriggervalue = 0.3;
        if (gamepad1.left_trigger > mintriggervalue || gamepad1.right_trigger > mintriggervalue) {
            if (gamepad1.left_trigger > mintriggervalue && !isIntakeMaxExtended()) {
                // going out
                if (!intakeReleased) {
                    isError = true; // hook must be released before extending intake
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
     *
     * @return true if intake was released
     */
    private boolean isIntakeReleased() {
        double pos = intakeHolder.getPosition();
        return pos > POS_INTAKE_RELEASE - 0.01;
    }

    /**
     * deenergize hook servo (if enabled)
     */
    private void deenergizeHook() {
        if (hookServo.isPwmEnabled()) {
            hookServo.setPwmDisable();
        }
    }

    /**
     * energize hook servo (if disabled)
     */
    private void energizeHook() {
        if (!hookServo.isPwmEnabled()) {
            hookServo.setPwmEnable();
        }
    }

    /**
     * Scaling function to convert gamepad input into motor power
     * @param x gamepad input
     * @return motor power
     */
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

    /**
     * This method sets up the servo to map 0-1 positions
     * into the available servo range.
     * The servo positions can be in any order, the first will be mapped to 0,
     * the second will be mapped to 1.
     * The servo will be set into the first (0) position.
     *
     * setPercentOpen() can be used afterwards to set servo position
     *
     * @param servo  - servo motor
     * @param zeroPos  - position between 0 and 1 that will be mapped to zero
     * @param onePos - position between 0 and 1 that will be mapped to one
     */
    static void setUpServo(Servo servo, double zeroPos, double onePos) {
        // scale t0 the range between inPos and outPos
        double min = Math.min(zeroPos, onePos);
        double max = Math.max(zeroPos, onePos);
        servo.scaleRange(min, max);

        // make sure in position = 0, out position = 1
        if (zeroPos > onePos) {
            servo.setDirection(Servo.Direction.REVERSE);
        } else {
            servo.setDirection(Servo.Direction.FORWARD);
        }

        setPercentOpen(servo, 0);
    }

    /**
     * Assuming closed/hold position is 0, open/release is 1, set percent open
     *
     * @param servo       - servo motor to use
     * @param percentOpen is the number between 0 and 1
     */
    static void setPercentOpen(Servo servo, double percentOpen) {
        servo.setPosition(percentOpen);
    }

}