package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    enum GoldPosition {
        left, center, right, undetected
    }

    GoldPosition goldPosition = GoldPosition.undetected;

    static final double POS_MARKER_FORWARD = 0.375;
    static final double POS_MARKER_UP = 0.7;
    static final double POS_MARKER_BACK = 0.75;

    static final double POS_HOOK_CLOSED = 0.5;
    static final double POS_HOOK_OPEN = 0.2;

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor liftMotor;

    private Servo hookServo;
    private Servo markerServo;
    private Servo intakeGateServo;
    private Servo boxServo;

    DistanceSensor rangeSensorFrontLeft;
    DistanceSensor rangeSensorBackLeft;
    DistanceSensor rangeSensorFrontRight;
    DistanceSensor rangeSensorBackRight;

    private TouchSensor liftTouch;

    boolean goldOnSide = false; //if gold isn't in the center
    boolean goldOnRight = false; //if gold on the right

    protected boolean depotSide() {
        return false;
    }

    @Override
    public void doRunOpMode() throws InterruptedException {

        startMillis = currentTimeMillis();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer,
        // so we create that first.
        long startMs = currentTimeMillis();
        initVuforia();
        long vuforiaMs = currentTimeMillis() - startMs;

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
            preRun();

            log("vuforia/tfod " + vuforiaMs + "/" + tfodMs);

            waitForStart();

            goldOnSide = false;
            goldOnRight = false;
            startMillis = currentTimeMillis();

            landing();
            rotateAndMoveGold();

            deliverTeamMarker();

            // if lift is not retracted fully, do retract it.
            retractLift();

            park();


            telemetry.addData("Gyro Heading", getGyroAngles().firstAngle);
            telemetry.addData("Distance FR", rangeSensorFrontRight.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance FL", rangeSensorFrontLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance BL", rangeSensorBackLeft.getDistance(DistanceUnit.INCH));
            telemetry.update();
            sleep(5000);
        } finally {
            if (tfod != null) {
                tfod.shutdown();
                tfod = null;
            }
        }
    }

    public void preRun() {

        motorLeft = hardwareMap.dcMotor.get("wheelsLeft");
        motorRight = hardwareMap.dcMotor.get("wheelsRight");

        rangeSensorFrontLeft = hardwareMap.get(DistanceSensor.class, "rangeFrontLeft");
        rangeSensorBackLeft = hardwareMap.get(DistanceSensor.class, "rangeBackLeft");
        rangeSensorFrontRight = hardwareMap.get(DistanceSensor.class, "rangeFrontRight");
        rangeSensorBackRight = hardwareMap.get(DistanceSensor.class, "rangeBackRight");

        // run by power
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // float zero power
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftTouch = hardwareMap.touchSensor.get("lift_touch");
        //retractLift(); - safety hazard
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        gyroInit();

        // make sure gyro is calibrated
        while (!this.isStarted() && !this.isStopRequested()) {
            telemetry.clearAll();
            logGyro(false);
            telemetry.addData("tfod", tfod != null);
            telemetry.update();
            idle();
        }
    }

    public void landing() throws InterruptedException {
        if (!opModeIsActive()) return;

        try {

            if (out == null) {
                out = new StringBuffer();
            }

            // detect where the gold is
            goldPosition = getGoldPosition();
            telemetry.addData("Gold", goldPosition);
            telemetry.update();

            log("1st detection");

            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (goldPosition == GoldPosition.undetected) {
                // second attempt to detect where the gold is
                liftMotor.setPower(-0.7);
                int lowerPos = DriverRover.LATCHING_POS / 5;
                liftMotor.setTargetPosition(-lowerPos);
                while (Math.abs(liftMotor.getCurrentPosition()) <= lowerPos) {
                    idle();
                }
                sleep(200);
                liftMotor.setPower(0);
                goldPosition = getGoldPosition();
                telemetry.addData("Gold", goldPosition);
                telemetry.update();
                log("2nd detection");
            }

            // lower the robot
            liftMotor.setPower(-1);
            liftMotor.setTargetPosition(-DriverRover.LATCHING_POS);
            while (Math.abs(liftMotor.getCurrentPosition()) <= DriverRover.LATCHING_POS_LOW) {
                idle();
            }
            sleep(200);
            liftMotor.setPower(0);
            sleep(200);

            // open hook

            intakeGateServo = hardwareMap.servo.get("intakeGate");
            intakeGateServo.setPosition(DriverRover.POS_GATE_CLOSED);
            boxServo = hardwareMap.servo.get("box");
            boxServo.setPosition(DriverRover.POS_BUCKET_PARKED);
            markerServo = hardwareMap.servo.get("markerServo");
            markerServo.setPosition(POS_MARKER_UP);
            hookServo = hardwareMap.servo.get("hookServo");
            hookServo.setPosition(POS_HOOK_OPEN);
            sleep(1500);


            if (!opModeIsActive()) return;

            // retract lift
            liftMotor.setPower(1);
            liftMotor.setTargetPosition(0);
            // let hook clear the handle
            while (opModeIsActive() && Math.abs(liftMotor.getCurrentPosition()) > DriverRover.LATCHING_POS_LOW) {
                idle();
            }
            sleep(200);

            // move forward where the gold is visible again
            moveWithErrorCorrection(0.5, 0.2, 7, new GyroErrorHandler(0));

            log("At the line");
        } finally {
            if (tfod != null) {
                tfod.shutdown();
                tfod = null;
            }
        }
    }

    public void rotateAndMoveGold() throws InterruptedException {
        if (!opModeIsActive()) return;

        if (goldPosition.equals(GoldPosition.right)) {
            goldOnRight = true;
            goldOnSide = true;
        } else if (goldPosition.equals(GoldPosition.left)) {
            goldOnSide = true;
        }

        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Lift Position", liftMotor.getCurrentPosition());

        double heading = 0;
        if (goldOnRight) {
            rotate(0.3, 30);
            heading = -35; // clockwise from 0 - negative heading
        } else if (goldOnSide) {
            rotate(-0.3, 30); //Gold is on the left
            heading = 35; // counterclockwise from 0 - positive heading
        }
        sleep(500);
        log("Rotated to gold");

        int distanceToCenterGold = 19; //distance is in inches
        int distanceToSideGold = 21; //distance is in inches

        int extraDistance = 0; //extraDistance is for the depot side only

        if (goldOnSide) {
            if (depotSide()) {
                extraDistance = 8;
            }
//            goCounts(0.5, inchesToCounts(distanceToSideGold+extraDistance));
            moveWithErrorCorrection(0.7, 0.2, distanceToSideGold + extraDistance, new GyroErrorHandler(heading));

        } else {
            if (depotSide()) {
                extraDistance = 29;
            }
//            goCounts(0.5, inchesToCounts(distanceToCenterGold + extraDistance));
            moveWithErrorCorrection(0.7, 0.2, distanceToCenterGold + extraDistance, new GyroErrorHandler(heading));

        }

        log("Moved mineral");

    }

    void deliverTeamMarker() throws InterruptedException {
        if (!opModeIsActive()) return;
        //if facing crater, we are done
        if (depotSide()) {
            if (goldOnSide) {
                double currentAngle = getGyroAngles().firstAngle;
                if (goldOnRight) {
                    //return to previous heading
                    rotate(-0.3, Math.abs(currentAngle) - 5);
////                    goCounts(0.5, inchesToCounts(13));
//                    moveWithErrorCorrection(0.7,0.2, 13, new GyroErrorHandler(0));
//                    rotate(-0.3, 20);
////                    goCounts(0.5, inchesToCounts(8));
//                    moveWithErrorCorrection(0.7,0.2, 8, new GyroErrorHandler(35));
                    moveWithErrorCorrection(0.7, 0.2, 11, new GyroErrorHandler(0));
                } else {
                    //return to previous heading
                    rotate(0.3, Math.abs(currentAngle) - 5);
////                    goCounts(0.5, inchesToCounts(13));
//                    moveWithErrorCorrection(0.7,0.2, 13, new GyroErrorHandler(0));
//                    rotate(0.3, 30);
////                    goCounts(0.5, inchesToCounts(8));
//                    moveWithErrorCorrection(0.7,0.2, 8, new GyroErrorHandler(-45));
                    moveWithErrorCorrection(0.7, 0.2, 23, new GyroErrorHandler(-45));
                }
            }

        } else {
            //in crater zone
            double heading;
            double distanceBack;
            double distanceToWall = 52; //longest distance from right position
            // come back
            if (goldOnSide) {
                // added 3 inches because the robot is landing closer to jewels
                distanceBack = 21.0/2.0 + 3;
                if (goldOnRight) {
                    heading = -35;
                } else {
                    // left
                    heading = 35;
                    distanceToWall -= 12;
                }
            } else {
                //center
                distanceBack = 19.0/2.0 + 3;
                heading = 0;
                distanceToWall -= 6;
            }
            moveWithErrorCorrection(-0.7, -0.2, distanceBack, new GyroErrorHandler(heading));
            double currentHeading = getGyroAngles().firstAngle;
            double driveHeading = 90;
            double angleToRotate = Math.abs(currentHeading - driveHeading) - 5; //small adjustment for over rotation
            // rotate toward the wall
            rotate(-0.3, angleToRotate);
            // drive toward the wall
            moveWithErrorCorrection(0.7, 0.2, distanceToWall, new GyroErrorHandler(driveHeading));
            // rotate along the wall
            currentHeading = getGyroAngles().firstAngle;
            driveHeading = 130;
            angleToRotate = Math.abs(driveHeading - currentHeading) - 5;
            rotate(-0.3, angleToRotate);
            log("At the wall");
            // drive along the wall using two range sensors for correction
            double inchesForward = 30;
//            // drive forward along the wall
//            boolean clockwiseWhenTooClose = false;
//            double inchesToWall = 5;
//            RangeErrorHandler errorHandler = new RangeErrorHandler (rangeSensorFrontRight,
//                    rangeSensorBackRight, inchesToWall, clockwiseWhenTooClose, driveHeading);
//            moveWithErrorCorrection(0.7, 0.2, inchesForward, errorHandler);
            // Drive a bit towards the wall using gyro
            moveWithErrorCorrection(0.7, 0.2, inchesForward, new GyroErrorHandler(driveHeading));
        }
        //Deliver team marker.
        markerServo.setPosition(POS_MARKER_FORWARD);
        sleep(1200);
        markerServo.setPosition(POS_MARKER_UP);
        sleep(1000);
        log("Delivered marker");

    }

    void park() {
        if (!opModeIsActive())
            return; // delivering marker is not implemented for crater zone yet

        double distanceToTravel;

        if (!depotSide()){
            distanceToTravel = 55;
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
            moveWithErrorCorrection(-0.7, -0.2, distanceToTravel, errorHandler);
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
            moveWithErrorCorrection(0.7, 0.2, inchesForward, new GyroErrorHandler(moveForwardHeading));
            double currentHeading = getGyroAngles().firstAngle;
            double parkHeading = -45;
            double angleToRotate = Math.abs(currentHeading - parkHeading) - 5; //small adjustment for over rotation
            // rotate to be along the wall
            rotate(rotatePower, angleToRotate);
            sleep(1000);

            double inchesToWall = 5;
            DistanceSensor rangeF = rangeSensorBackLeft;
            DistanceSensor rangeB = rangeSensorFrontLeft;
            boolean clockwiseWhenTooClose = false;

            moveWithErrorCorrection(-0.7, -0.2, distanceToTravel,
                    new RangeErrorHandler(rangeF, rangeB, inchesToWall, clockwiseWhenTooClose, parkHeading));
        }
        log("Parked");
    }

    void retractLift() {
        long startMs = currentTimeMillis();
        while (!liftTouch.isPressed() && currentTimeMillis() - startMs < 5000) {
            liftMotor.setPower(1); //positive power retracts the lift arm
            sleep(10);
        }
        liftMotor.setPower(0.0);
    }

    /**
     * rotates robot the given angle, give negative power for counterclockwise rotation and
     * positive power for clockwise rotation
     *
     * @param power
     * @param angle
     */
    void rotate(double power, double angle) {
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
     * @param behavior
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
            while (opModeIsActive() && tfod != null && currentTimeMillis() - startMs < 2000) {
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
                    int nr = 1;

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
    boolean isRight(float y) {
        return y < 800.0 / 3;
    }

    boolean isCenter(float y) {
        return y >= (800.0 / 3) && y <= (800.0 / 3) * 2;
    }

    boolean isLeft(float y) {
        return y > (800.0 / 3) * 2;
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
            out.append("# "); // start of the comment
            out.append(depotSide() ? "Depot" : "Creator");
            out.append("\n"); // end of comment
            // table header followed by new line
            out.append("Time,Step,Gyro,GoldPos,RangeFL,RangeBL,RangeFR,RangeBR,DriveLeft,DriveRight,\n");
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