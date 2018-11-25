package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.List;

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

    static final double POS_MARKER_FORWARD = 0.8;
    static final double POS_MARKER_UP = 0.45;
    static final double POS_MARKER_BACK = 0.2;

    static final double POS_HOOK_CLOSED = 0.5;
    static final double POS_HOOK_OPEN = 0.2;

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor liftMotor;

    private Servo hookServo;
    private Servo markerServo;

    DistanceSensor rangeSensorLeft;


    protected boolean depotSide() {
        return false;
    }

    @Override
    public void doRunOpMode() throws InterruptedException {

        preRun();

        waitForStart();
        startMillis = currentTimeMillis();
        landing();

        rotateAndMoveGold();

        sleep(2000);
    }

    public void preRun() {
        initializeTensorFlow();

        motorLeft = hardwareMap.dcMotor.get("wheelsLeft");
        motorRight = hardwareMap.dcMotor.get("wheelsRight");

        rangeSensorLeft = hardwareMap.get(DistanceSensor.class, "rangeSensor");

        // run by power
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // float zero power
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(-1);
        liftMotor.setTargetPosition(-DriverRover.LATCHING_POS);
        sleep(6000);
        liftMotor.setPower(0);
        sleep(200);

        hookServo = hardwareMap.servo.get("hookServo");
        hookServo.setPosition(POS_HOOK_OPEN);
        sleep(1000);
        markerServo = hardwareMap.servo.get("markerServo");
        markerServo.setPosition(POS_MARKER_BACK);
        liftMotor.setPower(1);
        liftMotor.setTargetPosition(0);
        sleep(1500);
        int distanceForward = inchesToCounts(7);
        goCounts(0.3, distanceForward);
    }

    public void rotateAndMoveGold() throws InterruptedException {
        if (!opModeIsActive()) return;
        boolean goldOnSide = false; //if gold block isn't in the center
        boolean goldOnRight = false;

        goldPosition = getGoldPosition();

        if (goldPosition.equals(GoldPosition.right)) {
            goldOnRight = true;
            goldOnSide = true;
        } else if (goldPosition.equals(GoldPosition.left)) {
            goldOnSide = true;
        }

        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Lift Position", liftMotor.getCurrentPosition());

        if (goldOnRight) {
            rotate(0.3, 30);
        } else if (goldOnSide) {
            rotate(-0.3, 30); //Gold is on the left
        }
        sleep(500);

        int distanceToCenterGold = inchesToCounts(19); //distance is in counts
        int distanceToSideGold = inchesToCounts(21); //distance is in counts

        int extraDistance = 0; //extraDistance is for the depot side only

        if (goldOnSide) {
            if (depotSide()) {
                extraDistance = inchesToCounts(8);
            }
            goCounts(0.5, distanceToSideGold + extraDistance);
        } else {
            if (depotSide()) {
                extraDistance = inchesToCounts(23);
            }
            goCounts(0.5, distanceToCenterGold + extraDistance);
        }

        //if facing crater, we are done
        if (depotSide()) {
            if (goldOnSide) {
                if (goldOnRight) {
                    double currentAngle = getGyroAngles().firstAngle;
                    //return to previous heading
                    rotate(-0.3, Math.abs(currentAngle));
                    goCounts(0.5, inchesToCounts(13));
                    rotate(-0.3, 20);
                    goCounts(0.5, inchesToCounts(8));
                } else {
                    double currentAngle = getGyroAngles().firstAngle;
                    //return to previous heading
                    rotate(0.3, Math.abs(currentAngle));
                    goCounts(0.5, inchesToCounts(13));
                    rotate(0.3, 30);
                    goCounts(0.5, inchesToCounts(8));
                }
            }
            //Deliver team marker.
            markerServo.setPosition(POS_MARKER_FORWARD);
            sleep(2000);
            markerServo.setPosition(POS_MARKER_UP);
            sleep(2000);
        } else {
            if (goldOnSide) {
                if (goldOnRight) {
                    rotate(-0.3, 20);
                } else {
                    rotate(0.3, 10);
                }
                goCounts(0.5, inchesToCounts(5));
            }
            markerServo.setPosition(POS_MARKER_FORWARD);
            sleep(2000);
        }
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
        telemetry.addData("current heading", originalHeading);
        telemetry.update();
        sleep(500);
        long start = new Date().getTime();
        motorRight.setPower(-power);
        motorLeft.setPower(power);
        while (opModeIsActive() && Math.abs(currentHeading - originalHeading) < angle) {
            currentHeading = getGyroAngles().firstAngle;
            //telemetry.addData("current heading", currentHeading);
            //telemetry.update();
        }
        motorRight.setPower(0);
        motorLeft.setPower(0);
        long end = new Date().getTime();
        currentHeading = getGyroAngles().firstAngle;
        telemetry.addData("current heading", currentHeading);
        telemetry.addData("rotate time", end - start);
        telemetry.update();
        sleep(500);
        currentHeading = getGyroAngles().firstAngle;
        telemetry.addData("current heading", currentHeading);
        telemetry.update();
        sleep(500);
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

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    void initializeTensorFlow() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer,
        // so we create that first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
            sleep(10000);
        }
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
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    GoldPosition getGoldPosition() {

        long startTime = currentTimeMillis();

        try {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
                sleep(1000);
            }
            while (opModeIsActive() && currentTimeMillis() - startTime < 3000) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getRecognitions();
                    if (updatedRecognitions != null) {
                        if (updatedRecognitions.size() < 1) continue;

                        // phone rotation is disabled
                        // the phone is mounted on the side
                        // top (left) is the lowest y coord - 0, bottom (right) is the highest - 800
                        Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                            @Override
                            public int compare(Recognition r1, Recognition r2) {
                                return (int) (r1.getBottom() - r2.getBottom());
                            }
                        });

                        float goldBottom = -1;
                        float silver1Bottom = -1;
                        float silver2Bottom = -1;

                        int numberOfGolds = 0;
                        int numberOfOthers = 0;
                        int nr = 1;

                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition != null) {
                                telemetry.addData(recognition.getLabel() + (nr++),
                                        "bottom: " + (int) recognition.getBottom() + "," +
                                                " conf: " + recognition.getConfidence() + ", (" +
                                                recognition.getImageWidth() + "," + recognition.getImageHeight()
                                );
                            }
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                numberOfGolds++;
                                goldBottom = recognition.getBottom();
                            } else {
                                if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                    if (silver1Bottom < 0) {
                                        silver1Bottom = recognition.getBottom();
                                    } else if (silver2Bottom < 0) {
                                        silver2Bottom = recognition.getBottom();
                                    } else {
                                        return GoldPosition.undetected;
                                    }
                                }
                                numberOfOthers++;
                            }
                        }

                        if (numberOfGolds > 1 || numberOfOthers > 2) {
                            continue;
                        }

                        if (goldBottom >= 0) {
                            if (isRight(goldBottom)) {
                                return GoldPosition.right;
                            } else if (isCenter(goldBottom)) {
                                return GoldPosition.center;
                            } else if (isLeft(goldBottom)) {
                                return GoldPosition.left;
                            } else {
                                return GoldPosition.undetected; //This should never happen
                            }
                        }

                        if (silver1Bottom < 0 && silver2Bottom < 0) {
                            continue;
                        }

                        if (!isLeft(silver1Bottom) && !isLeft(silver2Bottom)) {
                            return GoldPosition.left;
                        } else if (!isCenter(silver1Bottom) && !isCenter(silver2Bottom)) {
                            return GoldPosition.center;
                        } else if (!isRight(silver1Bottom) && !isRight(silver2Bottom)) {
                            return GoldPosition.right;
                        }

                        telemetry.addData("numberOfGolds", numberOfGolds);
                        telemetry.addData("numberOfOthers", numberOfOthers);

                        telemetry.addData("Time", (currentTimeMillis() - startMillis) / 1000);
                        telemetry.update();
                        sleep(100);
                    }
                }
            }
        } finally {
            if (tfod != null) {
                tfod.deactivate();
                tfod.shutdown();
            }
        }
        return GoldPosition.undetected;
    }

    /**
     * The image height is 800 pixels. Because the origin of the y axis is at the
     * top of the phone, the bottom coordinate of the leftmost mineral must be between
     * 0 and 266.
     *
     * @param y
     * @return
     */
    boolean isLeft(float y) {
        return y >= 0 && y <= 800.0 / 3;
    }

    boolean isCenter(float y) {
        return y > 800.0 / 3 && y <= (800.0 / 3) * 2;
    }

    boolean isRight(float y) {
        return y > (800.0 / 3) * 2 && y <= 800;
    }
}