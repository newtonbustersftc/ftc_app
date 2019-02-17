package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
import static org.firstinspires.ftc.teamcode.DriverRover.POS_INTAKE_HOLD;
import static org.firstinspires.ftc.teamcode.DriverRover.POS_INTAKE_RELEASE;
import static org.firstinspires.ftc.teamcode.DriverRover.setUpServo;


@Autonomous(name = "AutoRoverCrater", group = "Main")
public class AutonomousRover extends BaseAutonomous {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

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

    static final double POS_MARKER_FORWARD = 0.400;
    static final double POS_MARKER_UP = 0.75;
    static final double POS_MARKER_BACK = 0.775;

    static final double POS_HOOK_CLOSED = 0.5;
    static final double POS_HOOK_OPEN = 0.2;

    LynxModule lynxModule;

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor liftMotor;
    private DcMotor deliveryRotate;
    private DcMotor intakeExtend;

    private Servo hookServo;
    private Servo markerServo;
    private Servo intakeGateServo;
    private Servo boxServo;
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

    @Override
    public void doRunOpMode() throws InterruptedException {
        telemetry.addData("Initializing","Please wait");
        telemetry.update();

        if (isStopRequested()) { return; }

        startMillis = currentTimeMillis();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer,
        // so we create that first.
        long startMs = currentTimeMillis();
        initVuforia();
        long vuforiaMs = currentTimeMillis() - startMs;

        if (isStopRequested()) { return; }

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


            //waitForStart();

            goldOnSide = false;
            goldOnRight = false;

            logComment("init ms: " + (currentTimeMillis()-startMillis));
            startMillis = currentTimeMillis();

            landing();

            if (!rotateAndMoveGold()) {
                return;
            }
            if(shortCraterMode){
                shortCraterModePark();
            }
            else{
                deliverTeamMarker();
                park();
            }

            intakeHolder.setPosition(POS_INTAKE_RELEASE);

            //retractLift();
            while (opModeIsActive()) {
                telemetry.addData("Gyro Heading", getGyroAngles().firstAngle);
                telemetry.addData("Distance FR", rangeSensorFrontRight.getDistance(DistanceUnit.INCH));
                telemetry.addData("Distance BR", rangeSensorBackRight.getDistance(DistanceUnit.INCH));
                telemetry.addData("Distance FL", rangeSensorFrontLeft.getDistance(DistanceUnit.INCH));
                telemetry.addData("Distance BL", rangeSensorBackLeft.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } finally {
            if (tfod != null) {
                tfod.shutdown();
                tfod = null;
            }
        }
    }

    public void preRun() {

        if (isStopRequested()) { return; }

        intakeHolder = hardwareMap.servo.get("intakeHolder");
        intakeHolder.setPosition(POS_INTAKE_HOLD);

        Lights.setUpLights(hardwareMap);
        Lights.resetLights();

        this.delay = 0;

        //We only care about the delay or the short/long crater mode if we are on the crater side
        SharedPreferences prefs = getSharedPrefs(hardwareMap);
        if(!depotSide()) {
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
        if(depotSide()){
            logPrefix += "_depot";
            autoMode = AutonomousOptionsRover.AutoMode.DEPOT.toString();
        }
        else {
            logPrefix += shortCraterMode ? "_short_crater" : "_long_crater";
            autoMode = AutonomousOptionsRover.AutoMode.CRATER.toString();
        }
        editor.putString(AUTO_MODE_PREF, autoMode);
        editor.apply();

        motorLeft = hardwareMap.dcMotor.get("wheelsLeft");
        motorRight = hardwareMap.dcMotor.get("wheelsRight");
        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // brake on zero power
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (isStopRequested()) { return; }

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
        //retractLift(); // safety hazard - remove before match
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // reset encoders for the rotating delivery arm
        deliveryRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //reset encoders for the intake arm
        intakeExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // run by power
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int ntries;
        boolean success = false;
        for(ntries=0;!isStopRequested() && !success && ntries<2;ntries++) {
            long startMs = currentTimeMillis();
            success=gyroInit();
            logComment("gyro init "+(!success?"":"failed ")+"ms: "+(currentTimeMillis() - startMs));
        }
        if (!success || tfod == null) {
            Lights.red(true);
            if(tfod == null){
                Lights.yellow(true);
            }
        }
        else {
            Lights.green(true);
        }

        // make sure gyro is calibrated
        while (!this.isStarted() && !this.isStopRequested()) {
            telemetry.clearAll();
            telemetry.addData("Gyro success/tries",success+"/"+ntries);
            logGyro(false);

            // the delay applies only to crater side
            if (!depotSide()) { telemetry.addData("prefs: delay/short", delay + "/" + shortCraterMode); }
            telemetry.addData("FrontLeft", String.format("%.1f in", rangeSensorFrontLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("BackLeft", String.format("%.1f in", rangeSensorBackLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("FrontRight", String.format("%.1f in", rangeSensorFrontRight.getDistance(DistanceUnit.INCH)));
            telemetry.addData("BackRight", String.format("%.1f in", rangeSensorBackRight.getDistance(DistanceUnit.INCH)));

            telemetry.addData("tfod", tfod != null);

            telemetry.update();
            idle();
        }
    }

    void landing() throws InterruptedException {
        if (!opModeIsActive()) return;

        Lights.resetLights();

        try {

            if (out == null) {
                out = new StringBuffer();
            }

            boxServo = hardwareMap.servo.get("box");
            boxServo.setPosition(DriverRover.POS_BUCKET_PARKED);

            deliveryRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deliveryRotate.setTargetPosition(350);
            deliveryRotate.setPower(0.6);
            sleep(500);

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
                while (Math.abs(liftMotor.getCurrentPosition()) <= lowerPos) {
                    idle();
                }
                liftMotor.setPower(0);
                goldPosition = getGoldPosition();
                telemetry.addData("Gold", goldPosition);
                telemetry.update();
                log("2nd detection");
            }
            if (goldPosition == GoldPosition.undetected) {
                Lights.red(true);
            }
            else{
                Lights.red(false);
                Lights.green(true);
                if(goldPosition == GoldPosition.center){
                    Lights.yellow(true);
                } else if (goldPosition == GoldPosition.left) {
                    Lights.white(true);
                } else if (goldPosition == GoldPosition.right) {
                    Lights.blue(true);
                }
            }

            deliveryRotate.setTargetPosition(0);
            deliveryRotate.setPower(0.1);

            // lower the robot
            liftMotor.setTargetPosition(-DriverRover.LATCHING_POS);
            liftMotor.setPower(-1);
            while (Math.abs(liftMotor.getCurrentPosition()) <= DriverRover.LATCHING_POS-100) {
                idle();
            }
            liftMotor.setPower(0);
            sleep(200);

            markerServo = hardwareMap.servo.get("markerServo");
            //for moving the marker hand forward, use 1, for moving it back, use 0
            setUpServo(markerServo, AutonomousRover.POS_MARKER_BACK, AutonomousRover.POS_MARKER_FORWARD);

            // open hook
            hookServo = hardwareMap.servo.get("hookServo");
            hookServo.setPosition(POS_HOOK_OPEN);
            intakeGateServo = hardwareMap.servo.get("intakeGate");
            intakeGateServo.setPosition(DriverRover.POS_GATE_CLOSED);
            boxServo = hardwareMap.servo.get("box");
            boxServo.setPosition(DriverRover.POS_BUCKET_PARKED);
            sleep(1200);

            if (!opModeIsActive()) return;

            // retract lift
            liftMotor.setTargetPosition(0);
            liftMotor.setPower(1);
            // let hook clear the handle
            while (opModeIsActive() && Math.abs(liftMotor.getCurrentPosition()) > DriverRover.LIFT_POS_CLEAR) {
                idle();
            }
            liftMotor.setPower(0);

            // move forward where the gold is visible again
            checkForGyroError();
            moveWithErrorCorrection(0.5, 0.2, 7, new GyroErrorHandler(0));

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

        double currentHeading = getGyroAngles().firstAngle;
        double driveHeading = 0;
        deliveryRotate.setPower(0);
        double heading = 0;
        if(shortCraterMode && goldOnRight){
                driveHeading = 40;
                //rotate(0.3, 40);
                heading = -45; // clockwise from 0 - negative heading
        }
        else {
            if (goldOnRight) {
                driveHeading = 30;
                //rotate(0.3, 30);
                heading = -35; // clockwise from 0 - negative heading
            } else if (goldOnSide) {
                driveHeading = 30;
                //rotate(-0.3, 30); //Gold is on the left
                heading = 35; // counterclockwise from 0 - positive heading
            }
        }
        if(goldOnSide){
            double angleToRotate = Math.abs(currentHeading - driveHeading);
            double rotatePower = (goldOnRight)? 0.3 : -0.3;
            if(Math.abs(currentHeading) < 0.001){
            //rotate by encoder counts because we cannot rely on the gyro function
                int encoderCountsChange = 460;
                rotateByEncoderCounts(rotatePower, encoderCountsChange);
            } else {
                rotate(rotatePower, angleToRotate);
            }
        }
        sleep(200);
        log("Rotated to gold");

        int distanceToCenterGold = 19+2; //distance is in inches
        int distanceToSideGold = 21+2; //distance is in inches

        int extraDistance = 0; //extraDistance is for the depot side only

        if (goldOnSide) {
            if (depotSide()) {
                extraDistance = 8;
            }
            if(shortCraterMode && goldOnRight){
                extraDistance += 14;
            }
            checkForGyroError();
            moveWithErrorCorrection(0.7, 0.2, distanceToSideGold + extraDistance, new GyroErrorHandler(heading));

        } else {
            if (depotSide()) {
                extraDistance = 29;
            }
            checkForGyroError();
            moveWithErrorCorrection(0.7, 0.2, distanceToCenterGold + extraDistance, new GyroErrorHandler(heading));

        }
        log("Moved mineral");

        if (checkForGyroError()) {
            if (depotSide()) {
                //if gyro is not functioning, go back to landing position and stop
                moveWithErrorCorrection(-0.7, -0.2, distanceToCenterGold + extraDistance, new GyroErrorHandler(heading));
            }
            return false;
        } else {
            return true;
        }
    }

    void deliverTeamMarker() throws InterruptedException {
        if (!opModeIsActive()) return;
        Lights.resetLights();
        //if facing crater, we are done
        if (depotSide()) {
            if (goldOnSide) {
                double currentAngle = getGyroAngles().firstAngle;
                if (goldOnRight) {
                    //return to previous heading
                    rotate(-0.3, Math.abs(currentAngle) - 5);
                    moveWithErrorCorrection(0.7, 0.2, 11, new GyroErrorHandler(0));
                } else {
                    //return to previous heading
                    rotate(0.3, Math.abs(currentAngle) - 5);
                    moveWithErrorCorrection(0.7, 0.2, 23, new GyroErrorHandler(-45));
                }
            }

        } else {
            //in crater zone
            double heading;
            double distanceBack;
            double distanceToWall = 55; //longest distance from right position
            // come back
            if (goldOnSide) {
                // added 3 inches because the robot is landing closer to jewels
                distanceBack = 21.0/2.0 + 1;
                if (goldOnRight) {
                    heading = -35;
                } else {
                    // left
                    heading = 35;
                    distanceToWall -= 13;
                }
            } else {
                //center
                distanceBack = 19.0/2.0 + 1;
                heading = 0;
                distanceToWall -= 7;
            }

            // move back
            checkForGyroError();
            moveWithErrorCorrection(-0.7, -0.2, distanceBack, new GyroErrorHandler(heading));
            log("Moved back");

            double currentHeading = getGyroAngles().firstAngle;
            double driveHeading = 90;
            double angleToRotate = Math.abs(currentHeading - driveHeading) - 5; //small adjustment for over rotation
            // rotate toward the wall
            rotate(-0.3, angleToRotate);
            // delay if needed, delay is in seconds, sleep accepts milliseconds
            sleep (1000 * delay);
            log("Rotated to wall");
            // drive toward the wall
            checkForGyroError();
            moveWithErrorCorrection(0.7, 0.2, distanceToWall, new GyroErrorHandler(driveHeading));
            if (isRightFrontTooFarFromWall()) {
                //too far away from the wall
                Lights.red(true);
                motorLeft.setPower(0.2);
                motorRight.setPower(0.2);
                while (isRightFrontTooFarFromWall()) {idle();}
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }
            log("At the wall");

            // rotate along the wall
            currentHeading = getGyroAngles().firstAngle;
            driveHeading = 130;
            angleToRotate = Math.abs(driveHeading - currentHeading) - 5;
            rotate(-0.3, angleToRotate);
            log("Aligned with the wall");
            // drive along the wall using two range sensors for correction
            double inchesForward = 30;
            // drive forward along the wall
            boolean clockwiseWhenTooClose = false;
            double inchesToWall = 3;
            RangeErrorHandler errorHandler = new RangeErrorHandler (rangeSensorFrontRight,
                    rangeSensorBackRight, inchesToWall, clockwiseWhenTooClose, driveHeading);
            moveWithErrorCorrection(0.7, 0.2, inchesForward, errorHandler);
            checkForGyroError();
            //moveWithErrorCorrection(0.7, 0.2, inchesForward, new GyroErrorHandler(driveHeading));
        }
        //Deliver team marker.
        markerServo.setPosition(1);
        sleep(500);
        markerServo.setPosition(0);
        sleep(500);
        log("Delivered marker");

    }

    boolean isRightFrontTooFarFromWall() {
        double distance = rangeSensorFrontRight.getDistance(DistanceUnit.INCH);
        //if sensor misbehaves, it returns values larger than 300
        if (distance < 20 && distance > 4.5) {
            return true;
        } else {
            return false;
        }
    }

    void shortCraterModePark(){
        Lights.resetLights();
        if(goldOnSide){
            //rotating to heading 0
            double currentHeading = getGyroAngles().firstAngle;
            double angleToRotate = Math.abs(currentHeading);
            if(goldOnRight){
                //turn 45 degrees clockwise
                rotate(-0.3, angleToRotate + 40);
            }
            else{
                //turn 45 degrees counterclockwise
                rotate(0.3, angleToRotate);
            }
            intakeHolder.setPosition(POS_INTAKE_RELEASE);
        }

    }

    void park() {
        if (!opModeIsActive())
            return; // delivering marker is not implemented for crater zone yet
        Lights.resetLights();
        double distanceToTravel;

        if (!depotSide()){
            distanceToTravel = 59;
            double driveHeading = 135;

//            // rotate toward the wall
//            double currentHeading = getGyroAngles().firstAngle;
//            double angleToRotate = Math.abs(currentHeading - 140); //small adjustment for over rotation
//            rotate(-0.3, angleToRotate);

            boolean clockwiseWhenTooClose = true;
            double inchesToWall = 3;
            TEST = true;
            RangeErrorHandler errorHandler = new RangeErrorHandler (rangeSensorBackRight,
                    rangeSensorFrontRight, inchesToWall, clockwiseWhenTooClose, driveHeading);
            //usually use distance sensor not gyro
            boolean success = moveWithErrorCorrection(-0.7, -0.2, distanceToTravel, errorHandler);
            if (!success) return;
        } else {
            // depot side
            double moveForwardHeading;
            double inchesForward = 5;
            double rotatePower;

            if (goldOnSide) {
                if (goldOnRight) {
                    rotate(-0.3, 35);
                    rotatePower = 0.3;
                    inchesForward = 26;
                    distanceToTravel = 79;
                    moveForwardHeading = 45;
                } else {
                    rotatePower = 0.3;
                    moveForwardHeading = -45;
                    distanceToTravel = 78;
                }
            } else {
                rotatePower = 0.3;
                moveForwardHeading = 0;
                inchesForward = 5;
                distanceToTravel = 78;
            }
            // move forward a bit
            checkForGyroError();
            moveWithErrorCorrection(0.7, 0.2, inchesForward, new GyroErrorHandler(moveForwardHeading));
            double currentHeading = getGyroAngles().firstAngle;
            double parkHeading = -45;
            double angleToRotate = Math.abs(currentHeading - parkHeading) - 5; //small adjustment for over rotation
            // rotate to be along the wall
            rotate(rotatePower, angleToRotate);
            sleep(200);

            log("Out to park");

            double inchesToWall = 3;
            DistanceSensor rangeF = rangeSensorBackLeft;
            DistanceSensor rangeB = rangeSensorFrontLeft;

            boolean success = moveWithErrorCorrection(-0.7, -0.2, distanceToTravel,
                    new RangeErrorHandler(rangeF, rangeB, inchesToWall, false, parkHeading));
            if (!success) return;
        }
        log("Parked");
    }

    void retractLift() {
        long startMs = currentTimeMillis();
        while (!liftTouch.isPressed() && currentTimeMillis() - startMs < 4000) {
            liftMotor.setPower(1); //positive power retracts the lift arm
        }
        liftMotor.setPower(0.0);
    }

    boolean checkForGyroError(){
        if(isGyroError()){
            Lights.red(true);
            return true;
        }
        return false;
    }

    /**
     * rotates robot the given angle, give negative power for counterclockwise rotation and
     * positive power for clockwise rotation
     *
     * @param power
     * @param angle
     */
    void rotate(double power, double angle) {
        checkForGyroError();
        if (!opModeIsActive()) return;
        double originalHeading = getGyroAngles().firstAngle;
        double currentHeading = originalHeading;
//        telemetry.addData("current heading", originalHeading);
//        telemetry.update();
//        sleep(500);
//        long start = new Date().getTime();
        motorRight.setPower(-power);
        motorLeft.setPower(power);
        while (opModeIsActive() && Math.abs(currentHeading - originalHeading) < angle) {
            currentHeading = getGyroAngles().firstAngle;
            //telemetry.addData("current heading", currentHeading);
            //telemetry.update();
        }
        motorRight.setPower(0);
        motorLeft.setPower(0);
//        long end = new Date().getTime();
//        currentHeading = getGyroAngles().firstAngle;
//        telemetry.addData("current heading", currentHeading);
//        telemetry.addData("rotate time", end - start);
//        telemetry.update();
//        sleep(500);
        currentHeading = getGyroAngles().firstAngle;
        telemetry.addData("current heading", currentHeading);
        telemetry.update();
//        sleep(500);
    }

    //For when gyro fails, use the right wheel's encoder counts
    void rotateByEncoderCounts(double power, int encoderCountsChange) {
        checkForGyroError();
        if (!opModeIsActive()) return;
        int originalCounts = motorRight.getCurrentPosition();
        int currentCounts = originalCounts;
        motorRight.setPower(-power);
        motorLeft.setPower(power);
        while (opModeIsActive() && Math.abs(currentCounts - originalCounts) < encoderCountsChange) {
            currentCounts = motorRight.getCurrentPosition();
        }
        motorRight.setPower(0);
        motorLeft.setPower(0);
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
        DcMotor motor = motorLeft;

        int startPos = motor.getCurrentPosition();
        motorLeft.setPower(power);
        motorRight.setPower(power);

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
        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);
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
        motorLeft.setZeroPowerBehavior(behavior);
        motorRight.setZeroPowerBehavior(behavior);
    }

    /**
     * @return the selected wheel encoder count
     */
    int getWheelPosition() {
        return motorLeft.getCurrentPosition();
    }

    /**
     * @param motorPower power to go in straight line from -1 to 1
     * @param steerPower power to make adjustments clockwise
     */
    void steer(double motorPower, double steerPower) {
        double leftPower = motorPower;
        double rightPower = motorPower;
        if (leftPower > 0) {
            if (steerPower > 0)
                leftPower += steerPower;
            else
                rightPower -= steerPower;
        } else {
            if (steerPower > 0) {
                rightPower -= steerPower;
            } else {
                leftPower += steerPower;
            }
        }

        // Normalize power if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
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
     * into left motor encoder counts.
     *
     * @param inches
     * @return
     */
    static int inchesToCounts(double inches) {
        // motorLeft with power 0.3 was used for testing
        return (int) (1000 * inches / 11.5);
    }

//    void initializeTensorFlow() {
//        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer,
//        // so we create that first.
//        initVuforia();
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//            telemetry.update();
//            sleep(10000);
//        }
//    }

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
            while (opModeIsActive() && tfod != null && currentTimeMillis() - startMs < 1000) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() < 1) continue;

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
                    telemetry.addData("Time", (currentTimeMillis() - startMs) / 1000);
                    telemetry.update();
                    sleep(50);

                    if (numberOfGolds > 1 || numberOfOthers > 2) {
                        out.append(String.format("# number gold=%d, silver=%d\n", numberOfGolds, numberOfOthers));
                        continue;
                    }

                    out.append(String.format("# gold=%d, silver1=%d, silver2=%d\n", (int) goldY, (int) silver1Y, (int) silver2Y));

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
                        continue;
                    }


                    if (!isLeft(silver1Y) && !isLeft(silver2Y)) {
                        return GoldPosition.left;
                    } else if (!isCenter(silver1Y) && !isCenter(silver2Y)) {
                        return GoldPosition.center;
                    } else if (!isRight(silver1Y) && !isRight(silver2Y)) {
                        return GoldPosition.right;
                    }

                    sleep(50);
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
     * @param y
     * @return
     */
    boolean isLeft(float y) {
        return y < 800.0 / 3;
    }

    boolean isCenter(float y) {
        return y >= (800.0 / 3) && y <= (800.0 / 3) * 2;
    }

    boolean isRight(float y) {
        return y > (800.0 / 3) * 2;
    }

    void logComment(String message) {
        if (out == null) {
            out = new StringBuffer();
        }
        out.append("# "); // start of the comment
        out.append(message);
        out.append("\n"); // new line at the end
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
                motorLeft.getCurrentPosition(),
                motorRight.getCurrentPosition()));
    }
}