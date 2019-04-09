package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import static android.os.SystemClock.sleep;
import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.teamcode.AutonomousOptionsRover.AUTO_MODE_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptionsRover.getSharedPrefs;
import static org.firstinspires.ftc.teamcode.MecanumWheels.MIN_CLOCKWISE;
import static org.firstinspires.ftc.teamcode.MecanumWheels.MIN_FORWARD;
import static org.firstinspires.ftc.teamcode.MecanumWheels.MIN_RIGHT;

@TeleOp(name = "DriverRover", group = "Main")
public class DriverRover extends OpMode {
    //TODO: EDIT LATER ON
    private static final double MINPOWER = 0.1;
    private double DPAD_POWER = 0.2; // slow move power

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
    static final int DELIVERY_ROTATE_MARKER_POS = 1900; // arm to deliver marker
    static final int DELIVERY_ROTATE_MAX_POS = 1250;
    static final int DELIVERY_ROTATE_VERT_POS = 1190; // arm is vertical
    static final int DELIVERY_ROTATE_UP_POS = 1050;
    static final int DELIVERY_ROTATE_BEFORE_HOME_POS = 400;
    static final int DELIVERY_ROTATE_HORIZ_POS = 350; // arm is horizontal
    static final int DELIVERY_ROTATE_INTAKE_POS = 103; //position when box is sitting on platform
    static final int DELIVERY_ROTATE_CAMERA_POS = 250; //lowest position that allows camera to see minerals
    static final int DELIVERY_ROTATE_CLEAR_PLATFORM = 350; //lowest position for platform to fall

    // encode positions for the delivery extension motor
    // 0 - arm is not extended
    static int MAX_DELIVERY_EXTEND_POS = 1550;
    static int ALMOST_MAX_DELIVERY_EXTEND_POS = MAX_DELIVERY_EXTEND_POS - 200;
    static final int ALMOST_MIN_DELIVERY_EXTEND_POS = 200;

    static final double POS_IGATE_CLOSED = 0.8; //intake gate closed
    static final double POS_IGATE_OPEN = 0.4; //intake gate open

    static final double POS_DGATE_CLOSED = 0.78; //delivery gate closed
    static final double POS_DGATE_OPEN = 0.23; //delivery gate open

    static final double POS_FINGERS_PARKED = 0.70;
    static final double POS_FINGERS_FLIPPED = 0.31;

    static final double POS_INTAKE_HOLD = 0.55; //0.421;
    static final double POS_INTAKE_RELEASE = 0.62; // 0.65;
    static final double POS_INTAKE_RELEASE_EXTREME = 0.7;

    static final double POS_BUCKET_PARKED = 0.32;
    static final double POS_BUCKET_UP = 0.8;
    static final double POS_BUCKET_INTAKE = 0.37; // also used as drop position
    static final double POS_BUCKET_DROP = 0.35;

    MecanumWheels wheels;
    private DcMotor liftMotor;
    private DcMotor intakeExtend; //extend - positive, retract - negative
    private DcMotor deliveryExtend; //extend - positive, retract - negative
    private DcMotor deliveryRotate; // raise - positive, lower - negative (with reverse)

    private ServoImplEx hookServo;
    private Servo markerServo;
    private Servo intakeGateServo;
    private Servo deliveryGateServo;
    private Servo fingersServo;
    private Servo boxServo;
    private CRServo intakeWheelServo;
    private Servo intakeHolder;

    private TouchSensor liftTouch;
    private TouchSensor intakeExtendTouch;
    private TouchSensor deliveryExtendTouch;
    private TouchSensor deliveryDownTouch;
    private ColorSensor colorSensor;
    private BNO055IMU imu; //gyro

    enum ArcState {
        AT_CRATER, ALIGNING, MOVING_TO_CRATER, MOVING_TO_LAUNCHER, AT_LAUNCHER
    }

    private ArcState arcstate;

    //for now, we don't support robot reversing the forward direction
    private final boolean forward = true;

    private boolean hookReleased = true; //when true, the hook is released
    private boolean hookButtonPressed = false; //when true, the hook button is pressed down

    private boolean switchExtension = false;
    private static boolean fullExtension = true;

    private boolean switchCrater = false;
    private boolean ourCrater = false;
    private boolean isDepotAuto = false;

    private boolean leftBumper1Pressed = false;

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

    enum MineralTransferState{
        INTAKE, DELIVERY_GATE_OPEN, INTAKE_GATE_OPEN, TRANSFER, DELIVERY_GATE_CLOSED, DELIVERY
    }
    private MineralTransferState transferState;
    private long transferTime;

    boolean dgateBtnPressed = false;
    boolean igateBtnPressed = false;

    //current motor speeds
    private double currentForward;
    private double currentRight;
    private double currentClockwise;

    private static final double HALF_WIDTH = 7.5; //width between wheels is 15 inches

    // minimum and maximum radius of arc move
    // between crater and depot side launcher
    //private static final double ARC_MIN_RADIUS = 20.0;
    //private static final double ARC_MAX_RADIUS = 35.0;

    StringBuffer out;
    long loopEndTime = 0;

    //constants for automatic rotation
    static final double MAX_ROTATE_POWER = BaseAutonomous.MAX_ROTATE_POWER;
    static final double MIN_ROTATE_POWER = BaseAutonomous.MIN_ROTATE_POWER;
    static final double CLOSE_ANGlE = BaseAutonomous.CLOSE_ANGlE;
    static final double FAR_ANGLE = BaseAutonomous.FAR_ANGLE;
    static final double TOLERANCE_ANGLE = 1d; //TEST

    @Override
    public void init() {
        Lights.setUpLights(hardwareMap);

        wheels = new MecanumWheels(hardwareMap, telemetry, true);
        // front left wheel is on a chain, so the direction was reversed
        DcMotor fl = wheels.getMotor(MecanumWheels.Wheel.FL);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        intakeExtend = hardwareMap.dcMotor.get("intakeExtend");
        deliveryExtend = hardwareMap.dcMotor.get("deliveryExtend");
        deliveryRotate = hardwareMap.dcMotor.get("deliveryRotate");

        // run by speed
        wheels.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // touch sensors
        liftTouch = hardwareMap.touchSensor.get("lift_touch");
        intakeExtendTouch = hardwareMap.touchSensor.get("intakeExtendTouch");
        deliveryDownTouch = hardwareMap.touchSensor.get("deliveryDownTouch");
        deliveryExtendTouch = hardwareMap.touchSensor.get("deliveryExtendTouch");

        // float zero power
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        deliveryExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        deliveryRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        deliveryRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

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
        isDepotAuto = !ourCrater;

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
        intakeGateServo.setPosition(POS_IGATE_CLOSED);

        deliveryGateServo = hardwareMap.servo.get("deliveryGate");
        deliveryGateServo.setPosition(POS_DGATE_CLOSED);

        fingersServo = hardwareMap.servo.get("fingers");
        fingersServo.setPosition(POS_FINGERS_PARKED);

        intakeHolder = hardwareMap.servo.get("intakeHolder");
        //intakeHolder.setPosition(POS_INTAKE_HOLD);
        intakeReleased = isIntakeReleased();

        boxServo = hardwareMap.servo.get("box");
        //for bucket servo that delivers the minerals to the launcher
        setUpServo(boxServo, POS_BUCKET_INTAKE, POS_BUCKET_UP);

        intakeWheelServo = hardwareMap.crservo.get("rightIntake");

        transferState = MineralTransferState.INTAKE;
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


        //if auto, then no manual
//        if (gamepad2.a) {
//            toLauncher();
//        } else if (gamepad2.x) {
//            toCrater();
//        } else
        if (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.left_bumper) {
            doArc(gamepad1.right_stick_x);
        } else {
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
        long ct = currentTimeMillis();
        if(loopEndTime > 0) {
            logEntry((ct-startTime)+"," + (ct-loopEndTime));
        }
        loopEndTime = ct;
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

        if (gamepad2.left_bumper) {
            if (!igateBtnPressed) {
                intakeGateServo.setPosition(POS_IGATE_OPEN);
                sleep(30);
                fingersServo.setPosition(POS_FINGERS_FLIPPED);
                igateBtnPressed = true;
            }
        } else {
            if (igateBtnPressed) {
                fingersServo.setPosition(POS_FINGERS_PARKED);
                intakeGateServo.setPosition(POS_IGATE_CLOSED);
                igateBtnPressed = false;
            }
        }
        if (gamepad2.right_bumper) {
            if (!dgateBtnPressed) {
                deliveryGateServo.setPosition(POS_DGATE_OPEN);
                dgateBtnPressed = true;
            }
        } else {
            if (dgateBtnPressed) {
                deliveryGateServo.setPosition(POS_DGATE_CLOSED);
                dgateBtnPressed = false;
            }
        }

        int deliveryRotatePos = deliveryRotate.getCurrentPosition();
//        if (gamepad2.left_trigger > 0.5) {
//            transferToDelivery();
//        }

        //if the delivery arm is down, the bucket should be in intake position.
        // 0 corresponds to the intake position (or drop) 1 - to vertical position,
        //    when the delivery arm is up
        if (deliveryDownTouch.isPressed() || deliveryRotatePos < 100) {
            boxServo.setPosition(0);
        } else {
            if (deliveryRotatePos < DELIVERY_ROTATE_UP_POS) {
                boxServo.setPosition(((double) deliveryRotatePos - DELIVERY_ROTATE_INTAKE_POS) /
                        ((double)DELIVERY_ROTATE_UP_POS - DELIVERY_ROTATE_INTAKE_POS));
            }

            //when the arm is up, we want the be able to invoke trigger to drop minerals.
            if (deliveryRotatePos > DELIVERY_ROTATE_UP_POS) {
                if (gamepad2.right_trigger > 0.2) {
                    /*
                     * When dropping debris, servo position 1 when
                     * box is vertical and position 0 when box is down.
                     */
                    boxServo.setPosition(1 - (gamepad2.right_trigger-0.2)); //Uses analog trigger position

                    arcstate = ArcState.AT_LAUNCHER;
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
        int currentPos = deliveryRotate.getCurrentPosition();
        if (currentPos <= DELIVERY_ROTATE_INTAKE_POS) {
            deliveryRotate.setPower(0);
            deliveryArmState = DeliveryState.HOME;
            return;
        }


        switch (deliveryArmState) {
            case HOME:
                if (currentPos > DELIVERY_ROTATE_INTAKE_POS) {
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
                        deliveryRotate.setTargetPosition(DELIVERY_ROTATE_INTAKE_POS);
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
                transferState = MineralTransferState.INTAKE;
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
                        //if (fullExtension) extendDeliveryArm(1);
                        extendDeliveryArm(1);
                    }
                }
                break;
            case DELIVERY:
                //if (fullExtension) extendDeliveryArm(1);
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

    /**
     * Switches delivery arm maximum extension to half and back
     */
    private void switchExtension() {
        if (fullExtension) {
            MAX_DELIVERY_EXTEND_POS = MAX_DELIVERY_EXTEND_POS / 2;
            ALMOST_MAX_DELIVERY_EXTEND_POS = ALMOST_MAX_DELIVERY_EXTEND_POS / 2;
        } else {
            MAX_DELIVERY_EXTEND_POS = MAX_DELIVERY_EXTEND_POS * 2;
            ALMOST_MAX_DELIVERY_EXTEND_POS = ALMOST_MAX_DELIVERY_EXTEND_POS * 2;
        }
        fullExtension = !fullExtension;
    }

//     private void transferToDelivery(){
//        switch(transferState){
//            case INTAKE:
//                deliveryGateServo.setPosition(POS_DGATE_OPEN);
//                transferState = MineralTransferState.DELIVERY_GATE_OPEN;
//                transferTime = currentTimeMillis();
//                break;
//            case DELIVERY_GATE_OPEN:
//                if(currentTimeMillis() - transferTime > 400){
//                    intakeGateServo.setPosition(POS_IGATE_OPEN);
//                    transferState = MineralTransferState.INTAKE_GATE_OPEN;
//                    transferTime = currentTimeMillis();
//                }
//                break;
//            case INTAKE_GATE_OPEN:
//                if(currentTimeMillis() - transferTime > 400){
//                    transferState = MineralTransferState.TRANSFER;
//                }
//                break;
//            case TRANSFER:
//                if (!gamepad2.a) {
//                    fingersServo.setPosition(POS_FINGERS_FLIPPED);
//                } else {
//                    fingersServo.setPosition(POS_FINGERS_PARKED);
//                }
//                if(gamepad2.b){
//                    fingersServo.setPosition(POS_FINGERS_PARKED);
//                    intakeGateServo.setPosition(POS_IGATE_CLOSED);
//                    transferState = MineralTransferState.DELIVERY_GATE_CLOSED;
//                    transferTime = currentTimeMillis();
//                }
//                break;
//            case DELIVERY_GATE_CLOSED:
//                if(currentTimeMillis() - transferTime > 300) {
//                    deliveryGateServo.setPosition(POS_DGATE_CLOSED);
//                } else if (currentTimeMillis() - transferTime > 500) {
//                    transferState = MineralTransferState.DELIVERY;
//                }
//                break;
//            case DELIVERY:
//                break;
//        }
//     }

    //INTAKE, INTAKE_GATE_CLOSED, DELIVERY_GATE_OPEN, INTAKE_GATE_OPEN, TRANSFER, DELIVERY_GATE_CLOSED, DELIVERY

    /**
     *
     * @param coefficient between -1 and 1, controls direction and radius of arc
     */
    private void doArc(double coefficient) {

        double FORWARD_POWER = 0.5;
        double ARC_MAX_CLOCKWISE_POWER = .5; //at 0.5 forward power
        double ARC_MIN_CLOCKWISE_POWER = .3; //same

        double clockwisePower;
        double forwardPower;
        double p = (ARC_MAX_CLOCKWISE_POWER - ARC_MIN_CLOCKWISE_POWER) * Math.abs(gamepad1.right_stick_x) + ARC_MIN_CLOCKWISE_POWER;

        // outer motor is left between our crater and our depot side launcher
        // outer motor is right between opposite crater and our depot side launcher
        if (ourCrater) {
            //backward moving from our crater to depot side launcher
            //forward moving from depot side launcher to our crater
            int sign = coefficient < 0 ? 1 : -1;
            forwardPower = sign * FORWARD_POWER;
            clockwisePower = sign * p;
        } else {
            int sign = coefficient < 0 ? -1 : 1;
            forwardPower = sign * FORWARD_POWER;
            clockwisePower = -sign * p;
        }
        wheels.powerMotors(forwardPower, 0, clockwisePower);
    }


    /**
     * Arc moves between our or opposite alliance crater and depot side launcher,
     * driver's right stick x is proportional to the radius, bigger values - sharper turn
     * When moving from/to opposite alliance crater right wheel is outer (further from the center of the arc)
     * When moving from/to our crater left wheel is outer
     */
    private void toLauncher() {
        //AT_CRATER, ALIGNING, MOVING_TO_CRATER, MOVING_TO_LAUNCHER, AT_LAUNCHER
        switch (arcstate) {
            case AT_CRATER:
                arcstate = ArcState.ALIGNING;
                break;
            case ALIGNING:
                //continue rotating to heading
                double targetHeading = getHeadingParallelToWall();
                boolean doneRotating = rotateToHeading(targetHeading);

                if (doneRotating) {
                    //TODO: Calculate arc radius
                    arcstate = ArcState.MOVING_TO_LAUNCHER;
                }
                break;
            case MOVING_TO_CRATER:
                powerRotate(0);
                arcstate = ArcState.MOVING_TO_LAUNCHER;
                break;
            case MOVING_TO_LAUNCHER:

                //when going to launcher, match coefficient of driver-controlled mode
                doArc(ourCrater ? 0.5 : -0.5);

                boolean atLine = onLine(); //atLine returned by doArc

                if (atLine) {
                    //stop the robot by giving 0 power
                    powerRotate(0);

                    //rotate so directly facing launcher
                    rotateToHeading(getHeadingAtDepotLander());

                    arcstate = ArcState.AT_LAUNCHER;
                }
                break;
            case AT_LAUNCHER:
                break;
        }
    }

    private void toCrater() {
        //AT_CRATER, ALIGNING, MOVING_TO_CRATER, MOVING_TO_LAUNCHER, AT_LAUNCHER
        switch(arcstate) {
            case AT_LAUNCHER:
                boolean doneRotating = rotateToHeading(getHeadingAtDepotLander());
                if (doneRotating) {
                    arcstate = ArcState.MOVING_TO_CRATER;
                }
                break;
            case MOVING_TO_LAUNCHER:
                powerRotate(0);
                arcstate = ArcState.MOVING_TO_CRATER;
                break;
            case MOVING_TO_CRATER:
                doArc(ourCrater ? -0.5 : 0.5);
                break;
            case ALIGNING:
                break;
            case AT_CRATER:
                break;
        }
    }

    //TODO: DO STUFF HERE
    private boolean onLine() {
        return true;
    }

    private double getHeadingParallelToWall() {
        if (isDepotAuto) {
            return ourCrater ? -135 : 135;
        } else {
            return ourCrater ? -45 : -135;
        }
    }

    private double getHeadingAtDepotLander() {
        return isDepotAuto ? 0 : 90;
    }

    private void powerRotate(double clockwiseSpeed) {
        wheels.powerMotors(0,0, clockwiseSpeed);
    }

    /**
     * rotate to the gyro heading
     * @param targetHeading heading we want to rotate to
     * @return true if rotation is complete, false otherwise
     */
    private boolean rotateToHeading(double targetHeading) {
        double currentHeading = getGyroAngles().firstAngle;
        double delta = targetHeading - currentHeading;
        double rotateHeading = delta; //how much to rotate in what direction
        double currentPower;

        if (delta > 180) {
            rotateHeading = delta - 360;
        } else if (delta < -180) {
            rotateHeading = delta + 360;
        }

        //heading --> has direction
        //angle --> doesn't have direction
        double rotateAngle = Math.abs(rotateHeading);

        //if delta is positive --> rotate ccw
        //if delta is negative --> rotate cw
        double signFactor = delta < 0 ? -1 : 1;

        if (rotateAngle < TOLERANCE_ANGLE) {
            currentPower = 0;
        } else if (rotateAngle < CLOSE_ANGlE) {
            currentPower = MIN_ROTATE_POWER;
        } else if (rotateAngle > FAR_ANGLE) {
            currentPower = MAX_ROTATE_POWER;
        } else {
            //(A - MIN_A) / (P - MIN_P) = (MAX_A - MIN_A) / (MAX_P - MIN_P)
            currentPower = ((MAX_ROTATE_POWER - MIN_ROTATE_POWER) * (rotateAngle - CLOSE_ANGlE)) /
                    (FAR_ANGLE-CLOSE_ANGlE) + MIN_ROTATE_POWER;
        }

        powerRotate(currentPower * signFactor);

        return currentPower == 0;
    }

    private void controlWheels() {
        double clockwise = scaled(gamepad1.right_stick_x);
        // fine control with up and down overrides coarse control left and right
        // up - clockwise, down - counterclockwise
        if (Math.abs(gamepad1.right_stick_y) > 0.6)
            clockwise = (-gamepad1.right_stick_y / Math.abs(gamepad1.right_stick_y)) * MIN_CLOCKWISE;

        //We are using robot coordinates
        //D-pad is used for slow speed movements.
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            double forward = 0;
            double right = 0;
            if (gamepad1.dpad_up) {
                forward = DPAD_POWER;
            } else if (gamepad1.dpad_down) {
                forward = -DPAD_POWER;
            }
            if (gamepad1.dpad_right) {
                right = DPAD_POWER * 1.5;
            } else if (gamepad1.dpad_left) {
                right = -DPAD_POWER * 1.5;
            }
            driverPowerWheels(forward, right, clockwise);
        } else {
            double forward = -scaled(gamepad1.left_stick_y);
            double right = scaled(gamepad1.left_stick_x);
            driverPowerWheels(0.7*forward, right, 0.6*clockwise);
        }
    }


    /**
     * Grdually change the speed from current to the requested
     *
     * @param forward   requested forward from -1 to 1
     * @param right     requested right from -1 to 1
     * @param clockwise requested clockwise from -1 to 1
     */
    private void driverPowerWheels(double forward, double right, double clockwise) {

        currentForward = calculateNextSpeed(forward, currentForward, MIN_FORWARD);
        currentRight = calculateNextSpeed(right, currentRight, MIN_RIGHT);
        currentClockwise = calculateNextSpeed(clockwise, currentClockwise, MIN_CLOCKWISE);
        wheels.powerMotors(currentForward, currentRight, currentClockwise);
    }

    private double calculateNextSpeed(double speed, double currentSpeed, double minSpeed) {
        double STEP = 0.2;
        if (Math.abs(speed) < 0.1) {
            // cut power when going to 0
            STEP = 1;
        }

        double nextSpeed;

        if (Math.abs(currentSpeed - speed) <= STEP) {
            nextSpeed = speed;
        } else {
            if (currentSpeed < speed) {
                //speed is increasing
                nextSpeed = currentSpeed + STEP;
                if (nextSpeed > 1)
                    nextSpeed = 1;
                else if (nextSpeed == 0) {
                    nextSpeed = minSpeed;
                } else if (nextSpeed >= -minSpeed && nextSpeed < 0) {
                    nextSpeed = 0;
                }
            } else {
                //speed is decreasing
                nextSpeed = currentSpeed - STEP;
                if (nextSpeed < -1) {
                    nextSpeed = -1;
                } else if (nextSpeed == 0) {
                    nextSpeed = -minSpeed;
                } else if (nextSpeed <= minSpeed && nextSpeed > 0) {
                    nextSpeed = 0;
                }
            }
        }
        return nextSpeed;
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
            leftBumper1Pressed = true;
        } else {
            if(leftBumper1Pressed) {
                intakeHolder.setPosition(POS_INTAKE_RELEASE_EXTREME);
                intakeReleased = true;
                leftBumper1Pressed = false;
            }
        }

        // intake wheel forward and reverse
        if (gamepad1.y) {
            intakeWheelServo.setPower(-0.8); // reverse
        } else if (gamepad1.b || gamepad2.right_bumper) {
            //we want the intake wheel to help move the minerals to the back during transfer
            intakeWheelServo.setPower(0.8); // forward
        } else {
            intakeWheelServo.setPower(0.0);
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

                arcstate = ArcState.AT_CRATER;
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
        telemetry.addData("transfer state", transferState);
        telemetry.addData("box trigger", gamepad2.right_trigger);
//        telemetry.addData("intake extend / touch",
//                intakeExtend.getCurrentPosition() + "/" + intakeExtendTouch.isPressed());
        telemetry.addData("delivery rotate / touch / state",
                deliveryRotate.getCurrentPosition() + "/" +
                        deliveryDownTouch.isPressed() + "/" + deliveryArmState);
//        telemetry.addData("delivery extend / touch",
//                deliveryExtend.getCurrentPosition() + "/" + deliveryExtendTouch.isPressed());
//        telemetry.addData("lift / touch",
//                liftMotor.getCurrentPosition() + "/" + liftTouch.isPressed());
//        telemetry.addData("alpha", colorSensor.alpha());
//        if (intakeHolder != null) {
//            telemetry.addData("Holder Pos", intakeHolder.getPosition());
//        }
    }

    /**
     * This method sets up the servo to map 0-1 positions
     * into the available servo range.
     * The servo positions can be in any order, the first will be mapped to 0,
     * the second will be mapped to 1.
     * The servo will be set into the first (0) position.
     *
     * 0-1 range can be used afterwards with setPosition to set servo position
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

        servo.setPosition(0);
    }

    @Override
    public void stop() {
        if (fingersServo != null) {
            fingersServo.setPosition((POS_FINGERS_PARKED+POS_FINGERS_FLIPPED)/2);
        }
        Lights.disableLight();
        String logPrefix = "driver";
        try {
            if (out != null) {
                //log file without the time stamp to find it easier
                File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + logPrefix + ".txt");

                //saving the log file into a file
                OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
                outputStreamWriter.write(out.toString());
                outputStreamWriter.close();

                //log file with the time stamp for history
                String timestamp = new SimpleDateFormat("MMMdd_HHmm", Locale.US).format(new Date());
                file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + logPrefix + "_" + timestamp + ".txt");
                telemetry.clear();
                telemetry.addData("File", file.getAbsolutePath());
                telemetry.update();

                //saving the log file into a file
                outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
                outputStreamWriter.write(out.toString());
                outputStreamWriter.close();

            }
        } catch (Exception e) {
            telemetry.clear();
            telemetry.addData("Exception", "File write failed: " + e.toString());
            telemetry.update();
        }
    }

    private void logEntry(String message) {
        logEntry(message, false);
    }

    private void logEntry(String message, boolean isComment) {
        if (out == null) {
            out = new StringBuffer();
        }
        if (isComment) {
            out.append("# "); // start of the comment
        }
        out.append(message);
        out.append("\n"); // new line at the end
    }

    /**
     * Save current gyro heading, roll, and pitch into angles variable
     * angles.firstAngle is the heading
     * angles.secondAngle is the roll
     * angles.thirdAngle is the pitch
     * Heading: clockwise is decreasing, counterclockwise is increasing
     * Pitch: Lift side w/ light is decreasing, vice versa
     * Roll: Lift side w/ mini USB port is decreasing, vice versa
     */
    private Orientation getGyroAngles() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}