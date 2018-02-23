package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.ALLIANCE_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POSITION_PREF;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.ARM_SCREW_UP;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.JEWEL_ARM_DOWN;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.JEWEL_ARM_HOME;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.JEWEL_KICK_CENTER;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.JEWEL_KICK_LEFT;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.JEWEL_KICK_RIGHT;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.LEFT_HAND_IN_POS;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.LEFT_HAND_OUT_POS;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.RELIC_GRAB_HOLD;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.RELIC_GRAB_HOME;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.RELIC_GRAB_RELEASE;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.RELIC_ROTATE_DOWN;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.RELIC_ROTATE_UP;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.RIGHT_HAND_IN_POS;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.RIGHT_HAND_OUT_POS;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.resetEncoders;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.setPercentOpen;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.setUpServo;

/**
 * Created by Brandon on 10/22/2017.
 */

@Autonomous(name = "AutonomousOpMode", group = "Main")
public class AutonomousOpMode_Relic extends LinearOpMode {

    StringBuffer out = null; //variable holds information for logs

    RelicRecoveryVuMark vuMark; //holds the detected vuMark image

    //detected color of jewel
    enum Color {
        RED, BLUE, NONE
    }

    DcMotor relicScrew; //controls height of relic arm
    DcMotor relicArm; //controls length of relic arm

    DcMotor pusher; //pushes glyphs out of elevator

    Servo leftHand; //left side of glyph grabber
    Servo rightHand; //right side of glyph grabber

    ColorSensor sensorColor; //for jewel color detection
    Servo jewelArm; //controls rod with color sensor and jewel kicker
    Servo jewelKick; //removes jewel from jewel platform

    Servo relicRotate; //rotates the relic up/down
    Servo relicGrab; //releases/grabs relic

    //height of relic screw to pickup relic from floor (red/blue other position)
    static final int ARM_SCREW_GRAB_FROM_FLOOR_BLUE = 600;
    static final int ARM_SCREW_GRAB_FROM_FLOOR_RED = 325;

    //The width of the bin in inches
    static final double BWIDTH = 7.5;

    //The length of the side of the glyph in inches
    static final double GSIDE = 6;

    //The length of the robot in inches
    static final double RLENGTH = 17.75;

    // The distance from the front of the glyph to the center of the robot in inches
    static final double GLYPH_RCENTER = RLENGTH / 2 + 3;

    //The distance from the center of the robot to the wall in inches
    static final double WALL_RCENTER = 22;

    //The length of each floor tile in inches
    static final double TILE_LENGTH = 23.5;

    //The minimum distance you need to move forward from the balancing board to have enough room to rotate fully
    static final double MINCLEAR = 24.4;

    //slowest forward power that still moves the robot
    //used in calculating power gradients
    static final double MINIMUM_POWER = 0.2;

    static final double ROTATE_POWER = 0.3;

    //for VuMark detection
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private long vuMarkTime; //last time to detect VuMark image (for telemetry)

    private DigitalChannel relicTouchSensor; //Touch sensor at farthest back position on the relic arm
    private DigitalChannel screwTouchSensor; //Touch sensor at lowest position on relic arm screw
    private DigitalChannel liftTouchSensor; //Touch sensor at bottom position of glyph lift
    private DcMotor lift; //controls height of glyph lift

    MecanumWheels wheels; //controls four mecanum wheel motors

    //autonomous options
    private boolean isBlue; //true if on blue alliance
    private boolean isCornerPos; //true of robot starts on corner platform

    private BNO055IMU imu; // gyro
    Orientation angles; // angles from gyro

    long startMillis;

    /**
     * initialized servos and sets them to start positions
     */
    void autonomousStart() {
        leftHand = hardwareMap.servo.get("Left-Hand");
        setUpServo(leftHand, LEFT_HAND_IN_POS, LEFT_HAND_OUT_POS);
        rightHand = hardwareMap.servo.get("Right-Hand");
        setUpServo(rightHand, RIGHT_HAND_IN_POS, RIGHT_HAND_OUT_POS);
        jewelArm = hardwareMap.servo.get("Jewel-Arm");
        jewelArm.setPosition(JEWEL_ARM_HOME);
        jewelKick = hardwareMap.servo.get("Jewel-Kick");
        jewelKick.setPosition(JEWEL_KICK_RIGHT);
        relicRotate = hardwareMap.servo.get("Relic-Rotate");
        relicRotate.setPosition(RELIC_ROTATE_DOWN);
        relicGrab = hardwareMap.servo.get("Relic-Grab");
        relicGrab.setPosition(RELIC_GRAB_RELEASE);
    }

    /**
     * raises glyph to clear the glyph of the platform
     * lowers glyph to drop off in cryptobox
     *
     * @param doRaise if true, raises glyph, otherwise, lowers
     */
    void raiseGlyph(boolean doRaise) {
        int raisecounts = isBlue ? 950 : 650;
        if (doRaise) {
            int startPos = lift.getCurrentPosition(); // Gives us encoder counts before the move
            int currentPos = startPos;
            lift.setPower(0.6);
            while (Math.abs(currentPos - startPos) <= raisecounts && opModeIsActive()) {
                currentPos = lift.getCurrentPosition();
            }
        } else {
            lift.setPower(-0.5);
            boolean touchSensorReleased = liftTouchSensor.getState();
            while (touchSensorReleased && (opModeIsActive() || (!this.isStarted() && !this.isStopRequested()))) {
                touchSensorReleased = liftTouchSensor.getState();
                idle();
            }
        }
        lift.setPower(0);
    }

    /**
     * raises and lowers jewel arm
     *
     * @param endPos is final position of jewel arm servo
     */
    void moveJewelArm(double endPos) {
        double currentPos = jewelArm.getPosition();
        int c = 1;
        if (endPos < currentPos) {
            c = -1;
        }
        while (Math.abs(endPos - currentPos) >= 0.025 && opModeIsActive()) {
            currentPos = currentPos + c * 0.025;
            jewelArm.setPosition(currentPos);
            telemetry.addData("Arm Position: ", currentPos);
            telemetry.update();
            sleep(40);
        }
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
    Orientation getGyroAngles() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles;
    }

    /**
     * adds information about gyro angles and status to telemetry
     *
     * @param update if true, telemetry is updated right away
     */
    void logGyro(boolean update) {
        telemetry.addLine().
                addData("status", imu.getSystemStatus().toShortString()).
                addData("calib", imu.getCalibrationStatus().toString());

        getGyroAngles();
        telemetry.addLine().
                addData("heading", formatAngle(angles.angleUnit, angles.firstAngle)).
                addData("roll", formatAngle(angles.angleUnit, angles.secondAngle)).
                addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
        if (update) {
            telemetry.update();
        }
    }

    /**
     * formats gyro angle for telemetry
     *
     * @param angleUnit
     * @param angle
     * @return
     */
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    /**
     * formats angles to one decimal point precision
     * @param degrees angle in degrees
     * @return a string
     */
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * initializes gyro
     */
    private void gyroInit() {
        // see the calibration sample opmode;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);
    }

    /**
     * code executed in init before start button is pressed:
     * initializes motors and sensors,
     * initializes variables from autonomous options,
     * initializes gyro,
     * loops VuMark detect method,
     * displays telemetry,
     *
     * @return last detected VuMark
     */
    RelicRecoveryVuMark preRun() {

        relicTemplate = vuforiaInitialize();
        sensorColor = hardwareMap.get(ColorSensor.class, "Color-Sensor");

        relicScrew = hardwareMap.dcMotor.get("Relic Screw");
        relicArm = hardwareMap.dcMotor.get("Relic Arm");
        relicArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        screwTouchSensor = hardwareMap.digitalChannel.get("Touch-Screw");
        screwTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        relicTouchSensor = hardwareMap.digitalChannel.get("Touch-Sensor Relic");
        relicTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        boolean screwTouchReleased = screwTouchSensor.getState();
        if (screwTouchReleased) {
            telemetry.addLine("WARNING! ARM IS NOT DOWN!");
            relicScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            relicScrew.setPower(-1);
            while (screwTouchReleased && !this.isStopRequested()) {
                idle();
                screwTouchReleased = screwTouchSensor.getState();
            }
            relicScrew.setPower(0);
        }
        boolean relicTouchReleased = relicTouchSensor.getState();
        if (relicTouchReleased) {
            telemetry.addLine("WARNING! ARM IS NOT RETRACTED!");
            relicArm.setPower(-0.7);
            while (relicTouchReleased && !this.isStopRequested()) {
                idle();
                relicTouchReleased = relicTouchSensor.getState();
            }
            relicArm.setPower(0);
        }
        if (this.isStopRequested()) return RelicRecoveryVuMark.UNKNOWN;
        resetEncoders(relicScrew, true);
        resetEncoders(relicArm, true);
        relicArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicArm.setPower(0);
        relicScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relicScrew.setPower(0);

        pusher = hardwareMap.dcMotor.get("Pusher");

        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);

        String allianceString = prefs.getString(ALLIANCE_PREF, "");
        isBlue = allianceString.equals("blue");

        String startPosString = prefs.getString(START_POSITION_PREF, "");
        isCornerPos = startPosString.equals("corner");

        wheels = new MecanumWheels(hardwareMap, telemetry, !isBlue);
        lift = hardwareMap.dcMotor.get("Lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftTouchSensor = hardwareMap.digitalChannel.get("Touch-Sensor");
        liftTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        raiseGlyph(false);
        boolean touchSensorReleased = liftTouchSensor.getState();
        if (!touchSensorReleased) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        sleep(1000);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(0);

        gyroInit();

        vuMark = RelicRecoveryVuMark.UNKNOWN;
        // Check the VuMark and Deliver the Glyphs
        relicTrackables.activate(); // Starts looking for VuMarks
        while (!this.isStarted() && !this.isStopRequested()) {
            telemetry.clearAll();
            logGyro(false);

            telemetry.addData(ALLIANCE_PREF, allianceString + " " + startPosString);
            telemetry.addData("Color RGB", sensorColor.red() + " " + sensorColor.green() + " " + sensorColor.blue());
            vuMark = findVuMark();
            telemetry.addData("vuMark", vuMark);
            telemetry.addData("vuMarkTime", vuMarkTime);
            telemetry.update();
            idle();

        }

        relicTrackables.deactivate(); // Stop looking for VuMarks

        return vuMark;
    }

    /**
     * implementation of LinearOpMode runOpMode() method
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {

        relicGrab = hardwareMap.servo.get("Relic-Grab");
        relicGrab.setPosition(RELIC_GRAB_HOME);

        //Initialize instance variables and detect the VuMark
        RelicRecoveryVuMark vuMark = preRun();

        waitForStart();

        long startTime = (new Date()).getTime();
        startMillis = currentTimeMillis();

        //if the VuMark is undetected, just use center
        log("Start VuMark "+vuMark);
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.CENTER;
        }

        try {
            autonomousStart(); //Initialize the servos
            sleep(800);

            // Kick the Jewels
            raiseGlyph(true);
            jewelKick.setPosition(JEWEL_KICK_CENTER);
            moveJewelArm(JEWEL_ARM_DOWN);
            log("Jewel Arm Down"+colorString());
            if (!isBlue && isCornerPos) {
                grabRelic();
            } else {
                int armScrewHeight;
                if (isBlue) {
                    armScrewHeight = ARM_SCREW_GRAB_FROM_FLOOR_BLUE;
                } else if (isCornerPos) {
                    armScrewHeight = ARM_SCREW_UP;
                } else {
                    armScrewHeight = ARM_SCREW_GRAB_FROM_FLOOR_RED;
                }

                relicScrew.setTargetPosition(armScrewHeight);
                relicScrew.setPower(1);
            }

            //detect jewel color and remove opposite color ball from platform
            Color color = jewelColorCheck();
            log("Detected " + color + " Jewel"+colorString());
            if (color == Color.RED) //Colour 1 is red
            {
                jewelKick.setPosition(isBlue ? JEWEL_KICK_RIGHT : JEWEL_KICK_LEFT);
            } else if (color == Color.BLUE) {
                jewelKick.setPosition(isBlue ? JEWEL_KICK_LEFT : JEWEL_KICK_RIGHT);
            }
            sleep(1000);
            jewelKick.setPosition(JEWEL_KICK_CENTER);
            moveJewelArm(JEWEL_ARM_HOME);
            sleep(500);

            if (!relicScrew.isBusy()){
                relicScrew.setPower(0);
            }

            //deliver glyph to cryptobox
            log("On platform");
            deliverGlyph(vuMark);

            log("End");
            long timeSpentMs = currentTimeMillis() - startMillis;
            telemetry.addData("Time spent(ms)", timeSpentMs);

        } finally {
            try {
                try {
                    // make sure no power is applied to the screw
                    relicScrew.setPower(0);
                    relicScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    // digital servos will keep the last position
                    // wish to leave hands in the open position
                    setPercentOpen(rightHand, 1);
                    setPercentOpen(leftHand, 1);
                    sleep(100); // to let servos move
                } catch (Exception e) {}

                String name = "lastrun";
                //log file without the time stamp to find it easier
                File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + name + ".txt");

                //saving the log file into a file
                OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
                outputStreamWriter.write(out.toString());
                outputStreamWriter.close();

                //log file with the time stamp for history
                String timestamp = new SimpleDateFormat("MMMdd_HHmm", Locale.US).format(new Date());
                file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + name + "_" + timestamp + ".txt");
                telemetry.addData("File", file.getAbsolutePath());

                //saving the log file into a file
                outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
                outputStreamWriter.write(out.toString());
                outputStreamWriter.close();

            } catch (Exception e) {
                telemetry.addData("Exception", "File write failed: " + e.toString());
            }
        }

        while (opModeIsActive()) {

        }

    }

    /**
     * Vuforia initialization
     *
     * @return
     */
    private VuforiaTrackable vuforiaInitialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //TODO: In the line below, delete "cameraMonitorViewId" before using to speed up program.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // get Vuforia license key
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        parameters.vuforiaLicenseKey = prefs.getString("vuforiaLicenseKey", null);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        return relicTemplate;
    }

    /**
     * VuMark detection
     *
     * @return the detected VuMark
     */
    private RelicRecoveryVuMark findVuMark() {
        //vuMark will be the previously discovered vuMark if it can not detect anything
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        long vuStartTime = currentTimeMillis();

        while (vuMark == RelicRecoveryVuMark.UNKNOWN &&
                ((this.isStarted() && this.opModeIsActive()) || (!this.isStopRequested() && !this.isStarted()))) {

            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            // If the seeing takes too long, then just go with putting a block in the left one.
            if (currentTimeMillis() - vuStartTime > 5 * 1000) {
                break;
            }
        }
        long vuEndTime = currentTimeMillis();
        vuMarkTime = (vuEndTime - vuStartTime);
        //telemetry.addData("VuMark", "%s visible", vuMark);

        return vuMark;

    }

    /**
     * returns the ball colour that is on the right
     *
     * @return Color.RED, Color.BLUE, and Color.NONE if cannot be determined
     */
    public Color jewelColorCheck() {
        // Check for determinable color
        long startTime = currentTimeMillis();
        Color color = Color.NONE;
        while (color == Color.NONE && opModeIsActive()) {
            if (sensorColor.blue() > sensorColor.red()) {
                color = Color.BLUE;
            } else if (sensorColor.red() > sensorColor.blue()) {
                color = Color.RED;
            }
            if (currentTimeMillis() - startTime > 5 * 1000) { // seconds for cutoff
                break;
            }
        }
        // Return color
        return color;
    }

    /**
     * moves robot given number of inches using the gyro to keep us straight
     *
     * @param startPower starting forward power
     * @param heading    gyro heading
     * @param inches     distance forward
     * @param endPower   ending forward power
     * @return true if distance required was travelled, false otherwise
     * @throws InterruptedException
     */
    boolean moveByInchesGyro(double startPower, double heading, double inches, double endPower) throws InterruptedException {

        boolean forward = startPower > 0;

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int initialcount = wheels.getMotor(MecanumWheels.Wheel.RR).getCurrentPosition();
        double error, steerSpeed; // counterclockwise speed

        double inchesForGradient = 10;
        double slope = (endPower - startPower) / inchesToCounts(Math.min(inchesForGradient, inches), forward);
        double countsForGradient = (inches < inchesForGradient) ? 0 : Math.abs(inchesToCounts(inches - inchesForGradient, forward));

        double kp = 0.04; //experimental coefficient for proportional correction of the direction
        double countsSinceStart = Math.abs(wheels.getMotor(MecanumWheels.Wheel.RR).getCurrentPosition() - initialcount);
        double motorPower = startPower;
        while (opModeIsActive() && countsSinceStart < inchesToCounts(inches, forward)) {
            // error CCW - negative, CW - positive
            error = getRawHeadingError(heading);

            if (Math.abs(error) < 1) {
                steerSpeed = 0;
            } else {
                steerSpeed = kp * error / 4;
            }
            telemetry.addData("Error", error);
            telemetry.update();

            if (countsSinceStart > countsForGradient) {
                motorPower = slope * (countsSinceStart - inchesToCounts(inches, forward)) + endPower;
            }
            wheels.powerMotors(motorPower, 0, Range.clip(-steerSpeed, -1.0, 1.0));

            countsSinceStart = Math.abs(wheels.getMotor(MecanumWheels.Wheel.RR).getCurrentPosition() - initialcount);

        }
        wheels.powerMotors(0, 0, 0);

        return opModeIsActive();
    }

    /**
     * error for java detection
     *
     * @param heading desired heading
     * @return the error (difference between desired heading and actual heading)
     */
    double getRawHeadingError(double heading) {
        return (getGyroAngles().firstAngle - heading);
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
        //wheels.resetEncoders();
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor motor = wheels.getMotor(MecanumWheels.Wheel.RR);
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
            wheels.logEncoders();
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
        wheels.logEncoders();
        return finished;

    }

    /**
     * rotates robot the given angle, give negative power for counterclockwise rotation and
     * positive power for clockwise rotation
     *
     * @param power
     * @param angle
     */
    void rotate(double power, double angle) {
        double originalHeading = getGyroAngles().firstAngle;
        double currentHeading = originalHeading;
        long start = new Date().getTime();
        wheels.powerMotors(0, 0, power);
        while (opModeIsActive() && Math.abs(currentHeading - originalHeading) < angle) {
            currentHeading = getGyroAngles().firstAngle;
        }
        wheels.powerMotors(0, 0, 0);
        long end = new Date().getTime();
        telemetry.addData("rotate time", end - start);
    }

    /**
     * This method converts straight distance in inches
     * into front right motor encoder counts.
     *
     * @param inches
     * @param forward True: If the robot is moving forward. False: If the robot is moving backwards.
     * @return
     */
    static int inchesToCounts(double inches, boolean forward) {
        // RR wheel, power 0.4
        if (forward) {
            return (int) ((3000 * inches) / 34.5);
        } else {
            return (int) ((3000 * inches) / 35.0);
        }
    }

    /**
     * delivers glyph to cryptobox
     *
     * @param pos tells which cryptobox column to deliver to
     * @throws InterruptedException
     */
    void deliverGlyph(RelicRecoveryVuMark pos) throws InterruptedException {
        // Get in position
        if (isBlue) {
            // remember that mecanum wheels are configured with the backward direction
            // if you want to change it use  wheels.changeDirection();
            wheels.changeDirection();

            if (isCornerPos) {
                double totalDistance = 33.5;
                if (pos == RelicRecoveryVuMark.RIGHT) {
                    totalDistance = totalDistance + BWIDTH;
                } else if (pos == RelicRecoveryVuMark.LEFT) {
                    totalDistance = totalDistance - BWIDTH;
                }

                moveByInchesGyro(-0.3, 0, totalDistance, -MINIMUM_POWER);
                sleep(800);
                rotate(ROTATE_POWER, 108);
                sleep(800);
                goCounts(0.3, inchesToCounts(10.5, true));
            } else {
                double distanceToRelicPicking = TILE_LENGTH - 1;
                moveByInchesGyro(-0.3, 0, distanceToRelicPicking, -MINIMUM_POWER);
                // make sure the arm is at the right height
                while (relicScrew.isBusy()) {
                    idle();
                }
                relicScrew.setPower(0);
                log("Before relic pick up");
                grabRelic();
                log("After relic pick up");
                moveByInchesGyro(-0.3, 0, MINCLEAR + 2 - distanceToRelicPicking, -MINIMUM_POWER);
                log("Before rotating 90 degrees");
                rotate(-ROTATE_POWER, 90);
                log("After rotating 90 degrees");
                sleep(800);

                double totalDistance = 33 - TILE_LENGTH;
                if (pos == RelicRecoveryVuMark.RIGHT) {
                    totalDistance = totalDistance + BWIDTH;
                } else if (pos == RelicRecoveryVuMark.LEFT) {
                    totalDistance = totalDistance - BWIDTH;
                }

                if (totalDistance > 0) {
                    goCounts(0.3, inchesToCounts(totalDistance, true));
                }
                log("Before rotating to cryptobox");
                rotate(-ROTATE_POWER, 70);
                log("After rotating 70 to cryptobox");
                goCounts(0.3, inchesToCounts(10.5, true));
            }

        } else { // red
            double totalDistance = 29; //distance to the turn for the center bin

            if (!isCornerPos) {
                totalDistance = totalDistance - TILE_LENGTH;
                moveByInchesGyro(0.3, 0, MINCLEAR, MINIMUM_POWER);
                sleep(800);
                //Rotate to relic grabbing position.
                log("Before rotating 90 degrees");
                rotate(-ROTATE_POWER, 90);
                log("After rotating 90 degrees");
                sleep(800);
                log("Before relic pick up");
                grabRelic();
                log("After relic pick up");
            }


            if (pos == RelicRecoveryVuMark.RIGHT) {
                totalDistance = totalDistance - BWIDTH;
            } else if (pos == RelicRecoveryVuMark.LEFT) {
                totalDistance = totalDistance + BWIDTH;
            }
            // Rotate to cryptobox.
            goCounts(0.4, inchesToCounts(totalDistance, true));
            sleep(800);
            log("Before rotating to cryptobox");
            rotate(ROTATE_POWER, 68);
            log("After rotating to 68 cryptobox");
            sleep(800);

            goCounts(0.4, inchesToCounts(9, true));
            sleep(800);
        }

        if (!relicScrew.isBusy()){
            relicScrew.setPower(0);
        }

        // put the relic down and open hands
        raiseGlyph(false);
        setPercentOpen(rightHand, 1);
        setPercentOpen(leftHand, 1);
        sleep(300);

        log("Before push");

        // one last push
        wheels.powerMotors(0.3, 0, 0);
        sleep(700);
        wheels.powerMotors(0, 0, 0);

        goCounts(-0.4, 300);
        setPercentOpen(rightHand, 1);
        setPercentOpen(leftHand, 1);
        sleep(300);
    }

    /**
     * method for relic pickup in corner
     * 1. Lower hand, use relicRotate.setPosition(RELIC_ROTATE_DOWN)
     * 2. Open hand, use relicGrab.setPosition(RELIC_GRAB_RELEASE)
     * 3. Extend arm, use relicArm, go 1040 counts
     * 4. Close hand, use relicGrab.setPosition(RELIC_GRAB_HOLD)
     * 5. Retract arm, use relicArm, go back 1040 counts/or until touch button is pressed
     * 6. Raise hand, use relicRotate.setPosition(RELIC_ROTATE_UP)
     */
    void grabRelic() {
        relicRotate.setPosition(RELIC_ROTATE_DOWN);
        relicGrab.setPosition(RELIC_GRAB_RELEASE);

        int extendcounts = isBlue || isCornerPos ? 1040 : 800; //825
        if (isCornerPos) { extendcounts = 950; }

        //extending arm to relic
        moveRelicArm(1, extendcounts);

        relicGrab.setPosition(RELIC_GRAB_HOLD);
        sleep(800);
        log("Grabbed relic");

        //retract relic arm, halfway there rotates relic upward
        moveRelicArm(-1, extendcounts);
    }

    /**
     * extend/retract relic arm
     * when retracting, halfway through, rotates relic up, also raises arm to optimal height
     *
     * @param armPower positive = extending, negative = retracting
     * @param counts   extension/retraction distance in encoder counts
     */
    void moveRelicArm(double armPower, int counts) {
        boolean armMoving;
        boolean up = false;
        int startPosArm = relicArm.getCurrentPosition();
        // if we are retracting arm, we also want to move the arm up
        if (armPower < 0) {
            relicScrew.setTargetPosition(ARM_SCREW_UP);
            relicScrew.setPower(1);
        }
        relicArm.setPower(armPower);
        armMoving = true;
        while (opModeIsActive() && armMoving) {

            if (armPower < 0) { // arm retracting

                // we need to stop when touch button is pressed
                // relicTouchSensor.getState==true means the button is  NOT PRESSED
                boolean relicTouchPressed = !relicTouchSensor.getState();
                if (relicTouchPressed) {
                    relicArm.setPower(0);
                    armMoving = false;
                }

                //halfway point while retracting, we start rotating the relic up
                if (!up && (Math.abs(relicArm.getCurrentPosition() - startPosArm) > 0.5 * counts)) {
                    relicRotate.setPosition(RELIC_ROTATE_UP);
                    up = true;
                }
            }

            //when desired arm position is reached, the arm should stop moving
            if (armMoving && Math.abs(relicArm.getCurrentPosition() - startPosArm) >= counts) {
                relicArm.setPower(0);
                armMoving = false;
            }

            idle();
        }
        String action = armPower > 0 ? "Extended " : "Retracted ";
        log(action+counts+" counts");
        //precaution to make sure relicArm all stop
        relicArm.setPower(0);
    }

    /**
     * Add a log row to comma delimited table of steps
     * @param step make sure there is no commas in step string
     */
    void log(String step) {
        if (out == null) {
            out = new StringBuffer();
            out.append("# "); // start of the comment
            out.append(isBlue ? "BLUE" : "RED");
            out.append(isCornerPos ? " Corner" : " Other");
            out.append(colorString());
            out.append("\n"); // end of comment
            // table header followed by new line
            out.append("Time,Step,Gyro,Arm,Screw,FL,FR,RL,RR,\n");
        }
        out.append(currentTimeMillis()-startMillis).append(",");
        out.append(step).append(",");
        out.append(getGyroAngles().firstAngle).append(",");
        out.append(relicArm.getCurrentPosition()).append(",");
        out.append(relicScrew.getCurrentPosition()).append(",");
        out.append(wheels.getMotor(MecanumWheels.Wheel.FL).getCurrentPosition()).append(",");
        out.append(wheels.getMotor(MecanumWheels.Wheel.FR).getCurrentPosition()).append(",");
        out.append(wheels.getMotor(MecanumWheels.Wheel.RL).getCurrentPosition()).append(",");
        out.append(wheels.getMotor(MecanumWheels.Wheel.RR).getCurrentPosition()).append(",");
        out.append("\n"); // new line
    }

    String colorString() {
        return " Color RGB: " + sensorColor.red() + "/" + sensorColor.green() + "/" + sensorColor.blue();
    }
}
