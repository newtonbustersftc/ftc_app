package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.content.SharedPreferences;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import java.util.Locale;

import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.teamcode.AutonomousOptionsRover.AUTO_MODE_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptionsRover.CRATER_MODE_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptionsRover.DELAY_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptionsRover.getSharedPrefs;
import static org.firstinspires.ftc.teamcode.DriverRover.DELIVERY_ROTATE_CLEAR_PLATFORM;
import static org.firstinspires.ftc.teamcode.DriverRover.DELIVERY_ROTATE_INTAKE_POS;
import static org.firstinspires.ftc.teamcode.DriverRover.DELIVERY_ROTATE_MARKER_POS;
import static org.firstinspires.ftc.teamcode.DriverRover.DELIVERY_ROTATE_UP_POS;
import static org.firstinspires.ftc.teamcode.DriverRover.POS_BUCKET_INTAKE;
import static org.firstinspires.ftc.teamcode.DriverRover.POS_BUCKET_UP;
import static org.firstinspires.ftc.teamcode.DriverRover.POS_INTAKE_HOLD;
import static org.firstinspires.ftc.teamcode.DriverRover.POS_INTAKE_RELEASE;
import static org.firstinspires.ftc.teamcode.DriverRover.POS_INTAKE_RELEASE_EXTREME;


@Autonomous(name = "AutoRoverCrater", group = "Main")
public class AutonomousRover extends BaseAutonomous {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    static final double DRIVE_POWER = 0.65;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AbXEDef/////AAAAGU/yaxRV9EUDvGnzyyDBYXcKyDJ9pLb4hlfkcdjSSYUFWen3PVv+3CgFK9M2otdAxlvNvXj2mms/QU9yN/s5sZTgeh+iC39BihOEqC4I+0PYf1L8lqajM0dmEVbsfnGkDcCro+TnDT0vr6zTeJlprz7oNMqGQNZFKSUwEB9cMQR5eLOlUTyt4zhAYvX7Tz2E1cSAXlpVFBzo7/QOqaK7ex/iPo7h1m81KrFDI+yQ6WiveNWAev7kMnau8du//Jyr1I9D6e8QxeL0j/wLDxSrBGAqRnAHcyuQq7Eo39DIseEUu5k849py3hA0uKL8s7NYHg9LxTezwyhSk7cft8aGnCMJ9yMVJEUeTAnE2GEqySWF";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    long startMillis;
    private boolean firstLog = true;

    enum GoldPosition {
        left, center, right, undetected
    }

    GoldPosition goldPosition = GoldPosition.undetected;

    static final double POS_PLATFORM_PUSH = 0.600;
    static final double POS_PLATFORM_BACK = 0.775;

    static final double POS_HOOK_CLOSED = 0.5;
    static final double POS_HOOK_OPEN = 0.2;

    LynxModule lynxModule;

    MecanumWheels wheels;
    DcMotor motor; //This is the motor used for calibration
    DcMotor alternateMotor;

    private DcMotor liftMotor;
    DcMotor deliveryRotate;
    private DcMotor intakeExtend;

    private Servo hookServo;
    private Servo platformServo;
    private Servo intakeGateServo;
    Servo boxServo;
    private Servo intakeHolder;

    DistanceSensor rangeSensorFrontLeft;
    DistanceSensor rangeSensorBackLeft;
    DistanceSensor rangeSensorFrontRight;
    DistanceSensor rangeSensorBackRight;

    private TouchSensor liftTouch;

    boolean goldOnSide = false; //if gold isn't in the center
    boolean goldOnRight = false; //if gold on the right

    private int delay; // this is the delay for before coming to the depot area from the crater zone
    private boolean shortCraterMode;

    protected boolean depotSide() {
        return false;
    }

    static double imageHeight = 800; // Samsung's

    @Override
    public void doRunOpMode() throws InterruptedException {
        telemetry.addData("Initializing", "Please wait");
        telemetry.update();

        if (isStopRequested()) {
            return;
        }

        startMillis = currentTimeMillis();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer,
        // so we create that first.
        long startMs = currentTimeMillis();
        initVuforia();
        long vuforiaMs = currentTimeMillis() - startMs;

        if (isStopRequested()) {
            return;
        }

        try {
            long tfodMs = 0;
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                startMs = currentTimeMillis();
                initTfod();
                tfodMs = currentTimeMillis() - startMs;
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }

            //Initializes all motors.
            logComment(depotSide() ? "Depot" : "Crater");
            logComment("vuforia init ms: " + vuforiaMs);
            logComment("tfod init ms: " + tfodMs);
            preRun();

            // preRun is doing loop with telemetry update until stop or start is pressed
            // this is a recommended way to avid connection timeouts

            //waitForStart();

            // start or stop requested at this point
            if (isStopRequested()) {
                Lights.disableRed();
                return;
            }

            logComment("init ms: " + (currentTimeMillis() - startMillis));
            startMillis = currentTimeMillis();

            goldOnSide = false;
            goldOnRight = false;

            landing();

            if (!rotateAndMoveGold()) {
                return;
            }
            if (shortCraterMode) {
                shortCraterModePark();
            } else {
                deliverTeamMarker();
                park();
            }

            Lights.disableRed();
            Lights.green(true);
            sleep(600);
            log("Done");

            while (opModeIsActive()) {
                telemetry.addData("Gyro Heading", getGyroAngles().firstAngle);
                telemetry.addData("Distance FR", rangeSensorFrontRight.getDistance(DistanceUnit.INCH));
                telemetry.addData("Distance BR", rangeSensorBackRight.getDistance(DistanceUnit.INCH));
                telemetry.addData("Distance FL", rangeSensorFrontLeft.getDistance(DistanceUnit.INCH));
                telemetry.addData("Distance BL", rangeSensorBackLeft.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } finally {
            Lights.disableRed();
            if (tfod != null) {
                tfod.shutdown();
                tfod = null;
            }
        }
    }

    @SuppressLint("DefaultLocale")
    void preRun() {

        if (isStopRequested()) {
            return;
        }

        intakeHolder = hardwareMap.servo.get("intakeHolder");
        intakeHolder.setPosition(POS_INTAKE_HOLD);

        Lights.setUpLights(hardwareMap);
        Lights.resetLights();

        this.delay = 0;

        //We only care about the delay or the short/long crater mode if we are on the crater side
        SharedPreferences prefs = getSharedPrefs(hardwareMap);
        if (!depotSide()) {
            try {
                String delaystring = prefs.getString(DELAY_PREF, "");
                delaystring = delaystring.replace(" sec", "");
                this.delay = Integer.parseInt(delaystring);
            } catch (Exception e) {
                this.delay = 0;
            }
            try {
                String craterMode = prefs.getString(CRATER_MODE_PREF, AutonomousOptionsRover.CraterModes.LONG.toString());
                this.shortCraterMode = craterMode.equals(AutonomousOptionsRover.CraterModes.SHORT.toString());
            } catch (Exception e) {
                Lights.red(true);
            }
            logComment("prefs: delay " + delay + "ms, short mode: " + shortCraterMode);
        }

        SharedPreferences.Editor editor = prefs.edit();
        String autoMode;
        if (depotSide()) {
            logPrefix += "_depot";
            autoMode = AutonomousOptionsRover.AutoMode.DEPOT.toString();
        } else {
            logPrefix += shortCraterMode ? "_short_crater" : "_long_crater";
            autoMode = AutonomousOptionsRover.AutoMode.CRATER.toString();
        }
        editor.putString(AUTO_MODE_PREF, autoMode);
        editor.apply();

        wheels = new MecanumWheels(hardwareMap, telemetry);
        // front left wheel is on a chain, so the direction was reversed
        DcMotor fl = wheels.getMotor(MecanumWheels.Wheel.FL);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        wheels.resetEncoders();
        motor = wheels.getMotor(MecanumWheels.Wheel.FL);
        alternateMotor = wheels.getMotor(MecanumWheels.Wheel.FR);

        // brake on zero power
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (isStopRequested()) {
            return;
        }

        deliveryRotate = hardwareMap.dcMotor.get("deliveryRotate");
        deliveryRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // reverse so that positive power moves the delivery arm up
        deliveryRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeExtend = hardwareMap.dcMotor.get("intakeExtend");

        rangeSensorFrontLeft = hardwareMap.get(DistanceSensor.class, "rangeFrontLeft");
        rangeSensorBackLeft = hardwareMap.get(DistanceSensor.class, "rangeBackLeft");
        rangeSensorFrontRight = hardwareMap.get(DistanceSensor.class, "rangeFrontRight");
        rangeSensorBackRight = hardwareMap.get(DistanceSensor.class, "rangeBackRight");

        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftTouch = hardwareMap.touchSensor.get("lift_touch");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // reset encoders for the rotating delivery arm
        deliveryRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //reset encoders for the intake arm
        intakeExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // run by power
        wheels.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int ntries;
        boolean success = false;
        for (ntries = 0; !isStopRequested() && !success && ntries < 2; ntries++) {
            long startMs = currentTimeMillis();
            success = gyroInit();
            logComment("gyro init " + (success ? "" : "failed ") + "ms: " + (currentTimeMillis() - startMs));
        }
        if (!success || tfod == null) {
            Lights.red(true);
            if (tfod == null) {
                Lights.yellow(true);
            }
        } else {
            Lights.green(true);
        }

        // make sure gyro is calibrated
        while (!this.isStarted() && !this.isStopRequested()) {
            telemetry.clearAll();
            telemetry.addData("Gyro success/tries", success + "/" + ntries);
            logGyro(false);

            // the delay applies only to crater side
            if (!depotSide()) {
                telemetry.addData("prefs: delay/short", delay + "/" + shortCraterMode);
            }
            telemetry.addData("FrontLeft", String.format("%.1f in", rangeSensorFrontLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("BackLeft", String.format("%.1f in", rangeSensorBackLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("FrontRight", String.format("%.1f in", rangeSensorFrontRight.getDistance(DistanceUnit.INCH)));
            telemetry.addData("BackRight", String.format("%.1f in", rangeSensorBackRight.getDistance(DistanceUnit.INCH)));

            telemetry.addData("tfod", tfod != null);

            telemetry.update();
            idle();
        }
    }

    void landing() {
        if (!opModeIsActive()) return;

        Lights.resetLights();

        try {
            boxServo = hardwareMap.servo.get("box");
            boxServo.setPosition(DriverRover.POS_BUCKET_CAMERA);
            deliveryRotate.setTargetPosition(DELIVERY_ROTATE_CLEAR_PLATFORM);
            deliveryRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deliveryRotate.setPower(0.8);
            sleep(800);

            // detect where the gold is
            goldPosition = getGoldPosition();
            telemetry.addData("Gold", goldPosition);
            telemetry.update();

            log("1st detection");
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (goldPosition == GoldPosition.undetected) {
                Lights.red(true);
                // second attempt to detect where the gold is
                int lowerPos = Math.abs(DriverRover.LATCHING_POS - DriverRover.LATCHING_POS_LOW) / 2;
                liftMotor.setTargetPosition(-lowerPos);
                liftMotor.setPower(-0.7);
                while (Math.abs(liftMotor.getCurrentPosition()) <= lowerPos && goldPosition == GoldPosition.undetected) {
                    idle();
                }
                liftMotor.setPower(0);
                telemetry.update();
                goldPosition = getGoldPosition();
                telemetry.addData("Gold", goldPosition);
                telemetry.update();
                log("2nd detection");
            }
            if (goldPosition == GoldPosition.undetected) {
                Lights.red(true);
            } else {
                Lights.red(false);
                Lights.green(true);
                if (goldPosition == GoldPosition.center) {
                    Lights.yellow(true);
                } else if (goldPosition == GoldPosition.left) {
                    Lights.white(true);
                } else if (goldPosition == GoldPosition.right) {
                    Lights.blue(true);
                }
            }

            // lower the robot
            liftMotor.setTargetPosition(-DriverRover.LATCHING_POS);
            liftMotor.setPower(-1);
            while (Math.abs(liftMotor.getCurrentPosition()) <= DriverRover.LATCHING_POS - 100) {
                idle();
            }
            liftMotor.setPower(0);
            sleep(100);

            platformServo = hardwareMap.servo.get("markerServo");
            platformServo.setPosition(POS_PLATFORM_PUSH);

            // open hook
            hookServo = hardwareMap.servo.get("hookServo");
            hookServo.setPosition(POS_HOOK_OPEN);
            intakeGateServo = hardwareMap.servo.get("intakeGate");
            intakeGateServo.setPosition(DriverRover.POS_IGATE_CLOSED);
            sleep(600);
            platformServo.setPosition(POS_PLATFORM_BACK);

            if (!opModeIsActive()) return;

            // retract lift
            liftMotor.setTargetPosition(DriverRover.LIFT_POS_CLEAR);
            liftMotor.setPower(1);

            // move forward where the gold is visible again
            checkForGyroError();
            moveWithErrorCorrection(0.5, 0.2, 7, new GyroErrorHandler(0));

            // done retracting lift at this point
            liftMotor.setPower(0);

            log("At the line");
        } finally {
            if (tfod != null) {
                tfod.shutdown();
                tfod = null;
            }
        }
    }

    /**
     * Moves minerals
     *
     * @return true to continue, false to stop autonomous
     */
    boolean rotateAndMoveGold() {
        if (!opModeIsActive()) return false;

        Lights.resetLights();

        if (goldPosition.equals(GoldPosition.right)) {
            goldOnRight = true;
            goldOnSide = true;
        } else if (goldPosition.equals(GoldPosition.left)) {
            goldOnSide = true;
        }

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Lift Position", liftMotor.getCurrentPosition());

        double driveHeading = 0;

        double heading = 0; // center
        if (shortCraterMode && goldOnRight) {
            driveHeading = -40;
            heading = -45; // clockwise from 0 - negative heading
        } else {
            if (goldOnRight) {
                driveHeading = -35;
                heading = -35; // clockwise from 0 - negative heading
            } else if (goldOnSide) {
                driveHeading = 35;
                heading = 35; // counterclockwise from 0 - positive heading
            }
        }
        if (goldOnSide) {
            rotateToHeading(driveHeading);
        }
        sleep(100);
        log("Rotated to gold");

        int distanceToCenterGold = 18; //distance is in inches
        int distanceToSideGold = 20 + 1; //distance is in inches

        int extraDistance = 0; //extraDistance is for the depot side only

        if (goldOnSide) {
            if (shortCraterMode && goldOnRight) {
                extraDistance += 14;
            } else if (!goldOnRight) {
                // not clear why, but need extra distance on the left
                extraDistance = 2;
            }
            checkForGyroError();
            moveWithErrorCorrection(DRIVE_POWER, 0.2, distanceToSideGold + extraDistance, new GyroErrorHandler(heading));

        } else {
            checkForGyroError();
            moveWithErrorCorrection(DRIVE_POWER, 0.2, distanceToCenterGold + extraDistance, new GyroErrorHandler(heading));
        }
        log("Moved mineral");

        if (checkForGyroError()) {
            //if gyro is not functioning, go back to landing position and stop
            moveWithErrorCorrection(-DRIVE_POWER, -0.2, distanceToCenterGold + extraDistance, new GyroErrorHandler(heading));
            return false;
        } else {
            return true;
        }
    }

    void deliverTeamMarker() {
        if (!opModeIsActive()) return;
        Lights.resetLights();

        double heading;
        double distanceBack;
        double distanceToWall = depotSide() ? 46 : 44; // distance to wall center pos
        // come back
        if (goldOnSide) {
            // added a few inches because the robot is landing closer to jewels
            // also take care of 2 inches extra distance for left
            distanceBack = 20.0 / 2.0 + 1 + 1 + (goldOnRight ? 0 : 2);
            if (goldOnRight) {
                heading = -35;
                distanceToWall += 6;
            } else {
                // left
                heading = 35;
                distanceToWall -= 6;
            }
        } else {
            //center
            distanceBack = 18.0 / 2.0 + 1;
            heading = 0;
        }

        // move back
        checkForGyroError();
        moveWithErrorCorrection(-DRIVE_POWER, -0.2, distanceBack, new GyroErrorHandler(heading));
        log("Moved back");

        double driveHeading = depotSide()? 90 : -90; // 90: -90
        // rotate toward the wall
        rotateToHeading(driveHeading);
        if(!depotSide()){
            //TODO delay should depend on the place of the gold mineral
            // delay if needed, delay is in seconds, sleep accepts milliseconds
            sleep(1000 * delay);
        }

        log("Rotated to wall");
        // drive toward the wall
        checkForGyroError();

        int sign = depotSide()? 1 : -1;
        moveWithErrorCorrection( sign * DRIVE_POWER, sign * 0.2, distanceToWall, new GyroErrorHandler(driveHeading));
        log("At the wall");

        // rotate along the wall
        driveHeading = depotSide()? 135 : -45;
        rotateToHeading(driveHeading);
        log("Aligned with the wall");

        DistanceSensor s1 = depotSide()? rangeSensorBackRight : rangeSensorBackLeft;
        DistanceSensor s2 = depotSide()? rangeSensorFrontRight : rangeSensorFrontLeft;
        if (isTooFarFromWall(s1, s2)) {
            //too far away from the wall
            log("Too far");
            Lights.red(true);
            wheels.powerMotors(0, depotSide()? 0.3 : -0.3, 0);
            long stime = currentTimeMillis();
            while (currentTimeMillis() - stime < 2000 && isTooFarFromWall(s1, s2)) {
                idle();
            }
            wheels.powerMotors(0, 0, 0);
            Lights.red(false);
        }
        log("Close enough");

        // drive forward along the wall using two range sensors for correction
        double inchesBack = depotSide() ? 40 : 30;
        boolean clockwiseWhenTooClose = depotSide();
        double inchesToWall = 4;
        DistanceSensor leadingSensor = depotSide() ? rangeSensorBackRight : rangeSensorBackLeft;
        DistanceSensor followingSensor = depotSide() ? rangeSensorFrontRight : rangeSensorFrontLeft;
        RangeErrorHandler errorHandler = new RangeErrorHandler(leadingSensor,
                followingSensor, inchesToWall, clockwiseWhenTooClose, driveHeading);
        moveWithErrorCorrection(-DRIVE_POWER, -0.2, inchesBack, errorHandler);
        checkForGyroError();

        rotateToHeading(driveHeading);
        log("Aligned before drop");

        dropMarker();

        rotateToHeading(driveHeading);
        log("Aligned after drop");
    }

    public void dropMarker() {
        boxServo.setPosition(DriverRover.POS_BUCKET_INTAKE);

        deliveryRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //DELIVERY_ROTATE_INTAKE_POS, DELIVERY_ROTATE_UP_POS - arm positions
        //POS_BUCKET_INTAKE, POS_BUCKET_UP - bucket positions
        int armPos = deliveryRotate.getCurrentPosition();
        double boxPos;
        log("Before drop");
        while (armPos < DELIVERY_ROTATE_UP_POS) {
            armPos = armPos + 100;
            boxPos = POS_BUCKET_INTAKE + (POS_BUCKET_UP - POS_BUCKET_INTAKE) *
                    (armPos - DELIVERY_ROTATE_INTAKE_POS) /
                    (DELIVERY_ROTATE_UP_POS - DELIVERY_ROTATE_INTAKE_POS);
            deliveryRotate.setTargetPosition(armPos);
            deliveryRotate.setPower(0.4);
            boxServo.setPosition(boxPos);
            sleep(100);
        }
        while (armPos < DELIVERY_ROTATE_MARKER_POS) {
            armPos = armPos + 100;
            deliveryRotate.setTargetPosition(armPos);
            deliveryRotate.setPower(0.4);
            sleep(80);
        }
        boxServo.setPosition(0.6);
        sleep(1000);
        log("After drop");
        boxServo.setPosition(POS_BUCKET_UP);
        while (armPos > DELIVERY_ROTATE_UP_POS) {
            armPos = armPos - 100;
            deliveryRotate.setTargetPosition(armPos);
            deliveryRotate.setPower(0.6);
            sleep(80);
        }
        while (armPos > DELIVERY_ROTATE_INTAKE_POS + 100) {
            armPos = armPos - 100;
            boxPos = POS_BUCKET_INTAKE + (POS_BUCKET_UP - POS_BUCKET_INTAKE) *
                    (armPos - DELIVERY_ROTATE_INTAKE_POS) /
                    (DELIVERY_ROTATE_UP_POS - DELIVERY_ROTATE_INTAKE_POS);
            deliveryRotate.setTargetPosition(armPos);
            deliveryRotate.setPower(0.2);
            boxServo.setPosition(boxPos);
            sleep(80);
        }
        boxServo.setPosition(POS_BUCKET_INTAKE);
        deliveryRotate.setTargetPosition(DELIVERY_ROTATE_INTAKE_POS);
        deliveryRotate.setPower(0.1);

        log("Delivered marker");

        // use for testing autonomous
        //retractLift();
        //log("Lift retracted");
    }


    boolean isTooFarFromWall(DistanceSensor sensor1, DistanceSensor sensor2) {
        double distance = sensor1.getDistance(DistanceUnit.INCH);
        //if distance sensor misbehaves, it returns magic number 322.4409448818898 inches (8190 mm)
        if (distance >= 20) {
            distance = sensor2.getDistance(DistanceUnit.INCH);
        }
        logComment(distance + "");
        return distance < 20 && distance > 5;
    }

    void shortCraterModePark() {
        Lights.resetLights();
        if (goldOnSide) {
            //rotating to heading 0
            double currentHeading = getGyroAngles().firstAngle;
            double angleToRotate = Math.abs(currentHeading);
            if (goldOnRight) {
                //turn 45 degrees clockwise
                rotate(false, angleToRotate + 40);
            } else {
                //turn 45 degrees counterclockwise
                rotate(true, angleToRotate);
            }
            intakeHolder.setPosition(POS_INTAKE_RELEASE);
        }
    }

    void park() {
        if (!opModeIsActive())
            return; // delivering marker is not implemented for crater zone yet
        Lights.resetLights();
        double distanceToTravel;

        distanceToTravel = depotSide() ? 43 : 49; // 53;
        double driveHeading = depotSide() ? 135 : -45;

        boolean clockwiseWhenTooClose = !depotSide();
        double inchesToWall = 4;
        DistanceSensor leadingSensor = depotSide() ? rangeSensorFrontRight : rangeSensorFrontLeft;
        DistanceSensor followingSensor = depotSide() ? rangeSensorBackRight : rangeSensorBackLeft;
        RangeErrorHandler errorHandler = new RangeErrorHandler(leadingSensor,
                followingSensor, inchesToWall, clockwiseWhenTooClose, driveHeading);
        //usually use distance sensor not gyro
        intakeHolder.setPosition(POS_INTAKE_RELEASE_EXTREME);
        boolean success = moveWithErrorCorrection(DRIVE_POWER, 0.2, distanceToTravel, errorHandler);
        sleep(100);
        intakeExtend.setTargetPosition(-1200);
        intakeExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtend.setPower(0.35);
        if (!success) return;
        log("Parked");
    }

    // used for autonomous testing only
    void retractLift() {
        long startMs = currentTimeMillis();
        while (!liftTouch.isPressed() && currentTimeMillis() - startMs < 4000) {
            liftMotor.setPower(1); //positive power retracts the lift arm
        }
        liftMotor.setPower(0.0);
    }

    boolean checkForGyroError() {
        if (isGyroError()) {
            Lights.red(true);
            return true;
        }
        return false;
    }

    //For when gyro fails, use the right wheel's encoder counts
    void rotateByEncoderCounts(double power, int encoderCountsChange) {
        checkForGyroError();
        if (!opModeIsActive()) return;
        int originalCounts = motor.getCurrentPosition();
        int currentCounts = originalCounts;
        wheels.powerMotors(0, 0, power);
        while (opModeIsActive() && Math.abs(currentCounts - originalCounts) < encoderCountsChange) {
            currentCounts = motor.getCurrentPosition();
        }
        wheels.powerMotors(0, 0, 0);
    }


    /**
     * moves robot given number of encoder counts
     *
     * @param power  power to apply to all wheel motors
     * @param counts motor encoder counts
     * @return true if the robot travelled all the distance, false otherwise
     * @throws InterruptedException
     */
    boolean goCounts(double power, int counts) throws InterruptedException {
        if (!opModeIsActive()) return false;

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int startPos = motor.getCurrentPosition();
        wheels.powerMotors(power, 0, 0);

        int currentPos = startPos;
        //Detect robot being stuck: encoder count is not changing for half of a second
        long startTime = (new Date()).getTime();
        long msPassed = 0;
        double previousPos = currentPos;
        boolean finished = true; //this means the robot has traveled all the distance requested
        while (Math.abs(currentPos - startPos) < counts) {
            if (!opModeIsActive()) {
                finished = false;
                break;
            }
            idle();
            currentPos = motor.getCurrentPosition();

            // Stop if the wheel is stuck
            msPassed = (new Date()).getTime() - startTime;
            if (msPassed > 400) {
                if (Math.abs(currentPos - previousPos) < 50) {
                    finished = false;
                    break;

                }
                startTime = (new Date()).getTime();
                previousPos = currentPos;
            }
        }
        wheels.powerMotors(0, 0, 0);
        telemetry.addData("Counts Moved", Math.abs(motor.getCurrentPosition() - startPos));
        telemetry.update();
        return finished;
    }

    /**
     * set break or float behavior
     *
     * @param behavior zero power behavior for the wheels: BRAKE or FLOAT
     */
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        wheels.setZeroPowerBehavior(behavior);
    }

    /**
     * @return the selected wheel encoder count
     */
    int getWheelPosition() {
        return motor.getCurrentPosition();
    }

    double clockwiseK = 1.0;
    /**
     * @param motorPower power to go in straight line from -1 to 1
     * @param steerPower power to make adjustments clockwise
     */
    void steer(double motorPower, double steerPower) {
        wheels.powerMotors(motorPower, 0, Range.clip(clockwiseK*steerPower, -1.0, 1.0));
    }

    @Override
    void powerRotate(double rotateSpeed) {
        wheels.powerMotors(0, 0, rotateSpeed);
    }

    /**
     * @param inches
     * @param foward true when robot moves foward
     * @return
     */
    int inchesToCounts(double inches, boolean foward) {
        //it doesn't matter whether robot moves foward or backwards
        return inchesToCounts(inches);
    }

    /**
     * This method converts straight distance in inches
     * into calibration motor encoder counts.
     *
     * @param inches
     * @return
     */
    static int inchesToCounts(double inches) {
        // motorLeft with power 0.3 was used for testing
        //return (int) (1000 * inches / 11.5); // old robot - left motor counts
        //  1000 / 23.75 = counts per inch
        return (int) (1000 * inches / 23.75); // front-left mecanum wheel is used for calibration
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        //tfodParameters.useObjectTracker = false;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }

    GoldPosition getGoldPosition() {

        try {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
                sleep(500);
            }
            long startMs = currentTimeMillis();
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() < 1) return GoldPosition.undetected;

                // phone rotation is disabled
                // the phone is mounted on the side
                // top (right) is the lowest y coord - 0, bottom (left) is the highest - 800
                Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                    @Override
                    public int compare(Recognition r1, Recognition r2) {
                        return (int) (r1.getTop() - r2.getTop());
                    }
                });

                float goldY = -1;
                float silver1Y = -1;
                float silver2Y = -1;

                int numberOfGolds = 0;
                int numberOfOthers = 0;

                for (Recognition recognition : updatedRecognitions) {
                    if (recognition != null) {
//                            telemetry.addData(recognition.getLabel() + (nr++),
//                                    "y: " + (int) getMineralCenter(recognition) + "," +
//                                            " conf: " + recognition.getConfidence() + ", (" +
//                                            recognition.getImageWidth() + "," + recognition.getImageHeight()
//                            );
                        imageHeight = recognition.getImageHeight();

                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            numberOfGolds++;
                            goldY = getMineralCenter(recognition);
                        } else {
                            if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                if (silver1Y < 0) {
                                    silver1Y = getMineralCenter(recognition);
                                } else if (silver2Y < 0) {
                                    silver2Y = getMineralCenter(recognition);
                                }
                            }
                            numberOfOthers++;
                        }
                    }
                }

                telemetry.addData("numberOfGolds", numberOfGolds);
                telemetry.addData("numberOfOthers", numberOfOthers);
                telemetry.update();

                if (numberOfGolds > 1 || numberOfOthers > 2) {
                    logComment(String.format("number gold=%d, silver=%d", numberOfGolds, numberOfOthers));
                    if (numberOfOthers != 2) {
                        // if we have detected 2 silvers, we use them to find position of gold
                        return GoldPosition.undetected;
                    }
                }

                logComment(String.format("gold=%d, silver1=%d, silver2=%d", (int) goldY, (int) silver1Y, (int) silver2Y));

                if (goldY >= 0) {
                    if (isRight(goldY)) {
                        return GoldPosition.right;
                    } else if (isCenter(goldY)) {
                        return GoldPosition.center;
                    } else if (isLeft(goldY)) {
                        return GoldPosition.left;
                    } else {
                        return GoldPosition.undetected; //This should never happen
                    }
                }

                // if no gold is detected, need two silver recognitions to tell the position of gold
                // if two silver positions are not available, try again
                if (silver1Y < 0 || silver2Y < 0) {
                    // two silver positions are not available
                    //TODO: make random selection between two undeteced positions if only one silver is detected
                    return GoldPosition.undetected;
                }

                if (!isLeft(silver1Y) && !isLeft(silver2Y)) {
                    return GoldPosition.left;
                } else if (!isCenter(silver1Y) && !isCenter(silver2Y)) {
                    return GoldPosition.center;
                } else if (!isRight(silver1Y) && !isRight(silver2Y)) {
                    return GoldPosition.right;
                }
            }
        } finally {
            if (tfod != null) {
                tfod.deactivate();
            }
        }
        return GoldPosition.undetected;
    }

    float getMineralCenter(Recognition r) {
        return (r.getBottom() + r.getTop()) / 2;
    }

    /**
     * The image height is 800 pixels. Because the origin of the y axis is at the
     * top of the phone, the bottom coordinate of the rightmost mineral must be between
     * 0 and 266.
     *
     * @param y - y position
     * @return
     */
    boolean isLeft(float y) {
        return y < imageHeight / 3;
    }

    boolean isCenter(float y) {
        return y >= (imageHeight / 3) && y <= (imageHeight / 3) * 2;
    }

    boolean isRight(float y) {
        return y > (imageHeight / 3) * 2;
    }

    /**
     * Add a log row to comma delimited table of steps
     *
     * @param step make sure there is no commas in step string
     */
    void log(String step) {
        if (!opModeIsActive()) return;

        if (out == null) {
            out = new StringBuffer();
        }
        if (firstLog) {
            // table header followed by new line
            out.append("Time,Step,Gyro,GoldPos,RangeFL,RangeBL,RangeFR,RangeBR,DriveLeft,DriveRight,\n");
            firstLog = false;
        }

        out.append(String.format(Locale.US, "%5d,%22s,%4.1f,%s,%5.1f,%5.1f,%5.1f,%5.1f,%d,%d\n",
                currentTimeMillis() - startMillis,
                step,
                getGyroAngles().firstAngle, goldPosition,
                rangeSensorFrontLeft.getDistance(DistanceUnit.INCH),
                rangeSensorBackLeft.getDistance(DistanceUnit.INCH),
                rangeSensorFrontRight.getDistance(DistanceUnit.INCH),
                rangeSensorBackRight.getDistance(DistanceUnit.INCH),
                motor.getCurrentPosition(),
                alternateMotor.getCurrentPosition()));
    }
}