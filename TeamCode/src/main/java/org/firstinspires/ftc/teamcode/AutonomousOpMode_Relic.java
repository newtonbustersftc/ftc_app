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

import static org.firstinspires.ftc.teamcode.AutonomousOptions.ALLIANCE_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POSITION_PREF;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.resetEncoders;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.setPercentOpen;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.setUpServo;
/**
 * Created by Brandon on 10/22/2017.
 */

@Autonomous(name = "AutonomousOpMode", group = "Main")
public class AutonomousOpMode_Relic extends LinearOpMode {

    StringBuffer out;

    RelicRecoveryVuMark vuMark;

    enum Color{
        RED, BLUE, NONE
    }
    private DcMotor relicScrew;
    private DcMotor relicArm;

    private DcMotor pusher;

    ColorSensor sensorColor;
    Servo leftHand;
    public static final double LEFT_HAND_IN_POS = DriverOpMode_Relic.LEFT_HAND_IN_POS;
    public static final double LEFT_HAND_OUT_POS = DriverOpMode_Relic.LEFT_HAND_OUT_POS;

    Servo rightHand;
    public static final double RIGHT_HAND_IN_POS = DriverOpMode_Relic.RIGHT_HAND_IN_POS;
    public static final double RIGHT_HAND_OUT_POS = DriverOpMode_Relic.RIGHT_HAND_OUT_POS;

    Servo jewelArm;
    public static final double JEWEL_ARM_HOME = DriverOpMode_Relic.JEWEL_ARM_HOME;
    public static final double JEWEL_ARM_DOWN = DriverOpMode_Relic.JEWEL_ARM_DOWN;
    public static final double JEWEL_ARM_VERTICAL = DriverOpMode_Relic.JEWEL_ARM_VERTICAL;

    Servo jewelKick;
    public static final double JEWEL_KICK_RIGHT = DriverOpMode_Relic.JEWEL_KICK_RIGHT;
    public static final double JEWEL_KICK_LEFT = DriverOpMode_Relic.JEWEL_KICK_LEFT;
    public static final double JEWEL_KICK_CENTER = DriverOpMode_Relic.JEWEL_KICK_CENTER;

    Servo relicRotate;
    public static final double RELIC_ROTATE_UP = DriverOpMode_Relic.RELIC_ROTATE_UP; //holding the relic above the arm
    public static final double RELIC_ROTATE_DOWN = DriverOpMode_Relic.RELIC_ROTATE_DOWN; //holding the relic in place to grab or put down

    Servo relicGrab;
    public static final double RELIC_GRAB_HOLD = DriverOpMode_Relic.RELIC_GRAB_HOLD; //holding the relic
    public static final double RELIC_GRAB_RELEASE = DriverOpMode_Relic.RELIC_GRAB_RELEASE; //letting go of the relic

    public static final int ARM_SCREW_UP = DriverOpMode_Relic.ARM_SCREW_UP;
    public static final int ARM_SCREW_UP_BLUE_INIT = ARM_SCREW_UP/4;

    //The wisth of the bin in inches
    public static final double BWIDTH = 7.5;

    //The length of the side of the glyph in inches
    public static final double GSIDE = 6;

    //The length of the robot in inches
    public static final double RLENGTH = 17.75;

    // The distance from the front of the glyph to the center of the robot in inches
    public static final double GLYPH_RCENTER = RLENGTH/2 + 3;

    //The distance from the center of the robot to the wall in inches
    public static final double WALL_RCENTER = 22;

    //The length of each floor tile in inches
    public static final double TILE_LENGTH = 23.5;

    //The minimum distance you need to move forward from the balancing board to have enough room to rotate fully
    public static final double MINCLEAR = 24.4;

    public static final double MINIMUM_POWER = 0.15;

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    long vuMarkTime;

    private DigitalChannel relicTouchSensor; //Touch sensor at farthest back position on the relic arm
    private DigitalChannel touchSensor;
    DcMotor lift;

    MecanumWheels wheels;
    boolean isBlue;
    boolean isCornerPos;

    BNO055IMU imu; // gyro
    Orientation angles; // angles from gyro

    public void autonomousStart() {
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

    public void raiseGlyph(boolean doRaise) {
        int raisecounts = isBlue ? 950 : 650;
        if (doRaise) {
            int startPos = lift.getCurrentPosition(); // Gives us encoder counts before the move
            int currentPos = startPos;
            lift.setPower(0.6);
            while (Math.abs(currentPos - startPos) <= raisecounts &&  opModeIsActive()) {
                currentPos = lift.getCurrentPosition();
            }
        } else {
            lift.setPower(-0.5);
            boolean touchSensorReleased = touchSensor.getState();
            while (touchSensorReleased &&  (opModeIsActive() || (!this.isStarted() && !this.isStopRequested()))) {
                touchSensorReleased = touchSensor.getState();
                idle();
            }
        }
        lift.setPower(0);
    }

    public void moveJewelArm(double endPos) {
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
    public Orientation getGyroAngles(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles;
    }

    public void logGyro(boolean update){
        telemetry.addLine().
                addData("status", imu.getSystemStatus().toShortString()).
                addData("calib", imu.getCalibrationStatus().toString());

        getGyroAngles();
        telemetry.addLine().
                addData("heading", formatAngle(angles.angleUnit, angles.firstAngle)).
                addData("roll", formatAngle(angles.angleUnit, angles.secondAngle)).
                addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
        if(update){
            telemetry.update();
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void gyroInit(){


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.mode = BNO055IMU.SensorMode.GYRONLY;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled = true;
        //parameters.loggingTag = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);
    }

    public RelicRecoveryVuMark preRun() {
        out = new StringBuffer();

        relicTemplate = vuforiaInitialize();
        sensorColor = hardwareMap.get(ColorSensor.class, "Color-Sensor");

        relicScrew = hardwareMap.dcMotor.get("Relic Screw");
        relicArm = hardwareMap.dcMotor.get("Relic Arm");

        resetEncoders(relicScrew, true);
        resetEncoders(relicArm, true);

        relicArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicArm.setPower(0);
        relicScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicScrew.setPower(0);

        pusher = hardwareMap.dcMotor.get("Pusher");

        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);

        String allianceString = prefs.getString(ALLIANCE_PREF, null);
        isBlue = allianceString.equals("blue");

        String startPosString = prefs.getString(START_POSITION_PREF, null);
        isCornerPos = startPosString.equals("corner");

        wheels = new MecanumWheels(hardwareMap, telemetry, !isBlue);
        lift = hardwareMap.dcMotor.get("Lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        touchSensor = hardwareMap.digitalChannel.get("Touch-Sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        raiseGlyph(false);
        boolean touchSensorReleased = touchSensor.getState();
        if (!touchSensorReleased) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        sleep(1000);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(0);

        relicTouchSensor = hardwareMap.digitalChannel.get("Touch-Sensor Relic");
        relicTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        gyroInit();

        out.append("isBlue: "+isBlue+"\n");
        out.append("isCornerPos: "+isCornerPos+"\n");
        out.append("Color RGB: "+sensorColor.red() + " " + sensorColor.green() + " " + sensorColor.blue()+"\n");
        vuMark = RelicRecoveryVuMark.UNKNOWN;
        // Check the VuMark and Deliver the Glyphs
        relicTrackables.activate(); // Starts looking for VuMarks
        while(!this.isStarted() && !this.isStopRequested()) {
            telemetry.clearAll();
            logGyro(false);

            telemetry.addData(ALLIANCE_PREF, allianceString);
            telemetry.addData("isBlue", isBlue);
            telemetry.addData(START_POSITION_PREF, startPosString);
            telemetry.addData("isCornerPos", isCornerPos);
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

    @Override
    public void runOpMode() throws InterruptedException {

        RelicRecoveryVuMark vuMark = preRun();

        waitForStart();

        long startTime = (new Date()).getTime();

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
            if (!isBlue && isCornerPos) {
                grabRelic();
            } else {
                if (isBlue && !isCornerPos){
                    relicScrew.setPower(1);
                    int startpos = relicScrew.getCurrentPosition();
                    while (Math.abs(relicScrew.getCurrentPosition() - startpos) < ARM_SCREW_UP_BLUE_INIT){
                        idle();
                    }
                    relicScrew.setPower(0);
                }else {
                    relicScrew.setPower(1);
                    int startpos = relicScrew.getCurrentPosition();
                    while (Math.abs(relicScrew.getCurrentPosition() - startpos) < ARM_SCREW_UP){
                        idle();
                    }
//                    telemetry.addData("Color RGB", sensorColor.red() + " " + sensorColor.green() + " " + sensorColor.blue());
//                    telemetry.update();
//                    sleep(2000);
//                    telemetry.addData("Color RGB", sensorColor.red() + " " + sensorColor.green() + " " + sensorColor.blue());
//                    telemetry.update();
//                    sleep(1000);
                }
            }

            Color color = jewelColorCheck();
            out.append("Jewel Color RGB: "+sensorColor.red() + " " + sensorColor.green() + " " + sensorColor.blue()+"\n");
            out.append("Detected color: "+color+"\n");
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

            deliverGlyph(vuMark);

            long endTime = (new Date()).getTime();
            long timeSpentMs = endTime - startTime;
            out.append("Time spent (ms): "+ timeSpentMs +"\n");
            telemetry.addData("Time spent(ms)", timeSpentMs);

        } finally {
            try {
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


    private RelicRecoveryVuMark findVuMark() {
        //vuMark will be the previously discovered vuMark if it can not detect anything
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        long vuStartTime = System.currentTimeMillis();
        boolean opModeStarted = this.isStarted();
        boolean opModeActive = this.opModeIsActive();
        boolean isStopRequested = this.isStopRequested();
        while (vuMark == RelicRecoveryVuMark.UNKNOWN &&
                ((opModeStarted && opModeActive) || (!isStopRequested && !opModeStarted))) {

            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            // If the seeing takes too long, then just go with putting a block in the left one.
            if (System.currentTimeMillis() - vuStartTime > 5 * 1000) {
                break;
            }
        }
        long vuEndTime = System.currentTimeMillis();
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
        long startTime = System.currentTimeMillis();
        Color color = Color.NONE;
        while (color == Color.NONE && opModeIsActive()) {
            if (sensorColor.blue() > sensorColor.red()) {
                color = Color.BLUE;
            } else if (sensorColor.red() > sensorColor.blue()) {
                color = Color.RED;
            }
            if (System.currentTimeMillis() - startTime > 5 * 1000) { // seconds for cutoff
                break;
            }
        }
        // Return color
        return color;
    }

    /**
     * moves robot given number of inches using the gyro to keep us straight
     */
    public boolean moveByInchesGyro(double startPower, double heading, double inches, double endPower) throws InterruptedException {

        boolean forward = startPower > 0;

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int initialcount = wheels.getMotor(MecanumWheels.Wheel.FR).getCurrentPosition();
        double error, steerSpeed; // counterclockwise speed

        double inchesForGradient = 10;
        double slope = (endPower - startPower) / inchesToCounts(Math.min(inchesForGradient, inches), forward);
        double countsForGradient = (inches < inchesForGradient) ? 0 : Math.abs(inchesToCounts(inches - inchesForGradient, forward));

        double kp = 0.04; //experimental coefficient for proportional correction of the direction
        double countsSinceStart = Math.abs(wheels.getMotor(MecanumWheels.Wheel.FR).getCurrentPosition() - initialcount);
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

            countsSinceStart = Math.abs(wheels.getMotor(MecanumWheels.Wheel.FR).getCurrentPosition() - initialcount);

        }
        wheels.powerMotors(0, 0, 0);

        return opModeIsActive();
    }
    public double getRawHeadingError(double heading){
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
    public boolean goCounts(double power, int counts) throws InterruptedException {
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
                if (Math.abs(currentPos-previousPos) < 50) {
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
     * @param power
     * @param angle
     */
    void rotate (double power, double angle) {
        double originalHeading = getGyroAngles().firstAngle;
        double currentHeading = originalHeading;
        long start = new Date().getTime();
        wheels.powerMotors(0, 0, power);
        while (opModeIsActive() && Math.abs(currentHeading - originalHeading) < angle) {
            currentHeading = getGyroAngles().firstAngle;
        }
        wheels.powerMotors(0,0,0);
        long end = new Date().getTime();
        telemetry.addData("rotate time", end-start);
    }

    /**
     * This method converts straight distance in inches
     * into front right motor encoder counts.
     * @param inches
     * @param forward True: If the robot is moving forward. False: If the robot is moving backwards.
     * @return
     */
    public int inchesToCounts(double inches, boolean forward){
          // FL wheel, power 0.4
//        if (forward) {
//            return (int) ((3000*inches)/33.75);
//        } else {
//            return (int) ((3000*inches)/35.0);
//        }
        // RR wheel, power 0.4
        if (forward) {
            return (int) ((3000*inches)/35.00);
        } else {
            return (int) ((3000*inches)/35.75);
        }
    }

    public void deliverGlyph (RelicRecoveryVuMark pos) throws InterruptedException {
        // Get in position
        if (isBlue) {
            // park from the platform
            // goCounts(0.4, isCornerPos ? 3000 : 2900);

            // remember that mecanum wheels are configured with the backward direction
            // if you want to change it use  wheels.changeDirection();
            wheels.changeDirection();

            if (isCornerPos){
                double totalDistance = 33.5;
                if (pos == RelicRecoveryVuMark.RIGHT) {
                    totalDistance = totalDistance + BWIDTH;
                } else if (pos == RelicRecoveryVuMark.LEFT) {
                    totalDistance = totalDistance - BWIDTH;
                }

                moveByInchesGyro(-0.3, 0, totalDistance, -MINIMUM_POWER);
                sleep(800);
                rotate(0.3, 108);
                sleep(800);
                goCounts(0.3, inchesToCounts(10.5, true));
            } else {
                moveByInchesGyro(-0.3, 0, TILE_LENGTH, -MINIMUM_POWER);
                grabRelic();
                moveByInchesGyro(-0.3, 0, MINCLEAR+2-TILE_LENGTH, -MINIMUM_POWER);
                rotate(-0.3, 90);
                sleep(800);

                double totalDistance = 33-TILE_LENGTH;
                if (pos == RelicRecoveryVuMark.RIGHT) {
                    totalDistance = totalDistance + BWIDTH;
                } else if (pos == RelicRecoveryVuMark.LEFT) {
                    totalDistance = totalDistance - BWIDTH;
                }

                if(totalDistance > 0) {
                    goCounts(0.3, inchesToCounts(totalDistance, true));
                }
                rotate(-0.3, 72);
                goCounts(0.3, inchesToCounts(10.5, true));
            }

        } else { // red

            double totalDistance = 29; //distance to the turn for the center bin

            if (!isCornerPos) {
                totalDistance = totalDistance - TILE_LENGTH;
                moveByInchesGyro(0.3, 0, MINCLEAR, MINIMUM_POWER);
                sleep(800);
                if (vuMark != RelicRecoveryVuMark.RIGHT) {
                    rotate(-0.3, 93);
                    sleep(800);
                }
            }


            if (pos == RelicRecoveryVuMark.RIGHT) {
                totalDistance = totalDistance - BWIDTH;
            } else if (pos == RelicRecoveryVuMark.LEFT) {
                totalDistance = totalDistance + BWIDTH;
            }
            if (isCornerPos || vuMark != RelicRecoveryVuMark.RIGHT) {
                goCounts(0.4, inchesToCounts(totalDistance, true));
                sleep(800);
                rotate(0.3, 72);
                sleep(800);
            } else { // optimization for red other going to right bin
                rotate(-0.3, 19);
                sleep(800);
            }
            goCounts(0.4, inchesToCounts(9, true));
            sleep(800);
        }

        // put the relic down and move out
        raiseGlyph(false);
        setPercentOpen(rightHand, 1);
        setPercentOpen(leftHand, 1);
        sleep(800);

        // one last push
        wheels.powerMotors(0.3, 0, 0);
        sleep(750);
        wheels.powerMotors(0, 0, 0);
        sleep(800);

        // jk another push
        pusher.setPower(0.4);
        sleep(800);
        pusher.setPower(-0.4);

        goCounts(-0.4, 300);
        sleep(500);
        pusher.setPower(0);
    }

//    public void deliverGlyphV1 (RelicRecoveryVuMark pos) throws InterruptedException {
//
//        if (isBlue || !isCornerPos) {
//            return;
//        }
//
//        //For robot to get off the platform and clear the turn it needs to drive initialDistance
//        //double initialDistance = RLENGTH+((TILE_LENGTH-RLENGTH)/2)+(Math.sqrt(2)-1)*RLENGTH/2;
//
//        // red corner position
//        double totalDistance = TILE_LENGTH + GSIDE/4;
//        double distanceAfterRotation = WALL_RCENTER-GLYPH_RCENTER;
//
//        if (pos == RelicRecoveryVuMark.RIGHT) {
//        } else if (pos == RelicRecoveryVuMark.CENTER) {
//            totalDistance = totalDistance + BWIDTH;
//        } else if (pos == RelicRecoveryVuMark.LEFT) {
//            totalDistance = totalDistance + BWIDTH*2;
//        }
//        int totalCounts = inchesToCounts(totalDistance, true);
//        goCounts(0.4, totalCounts);
//        //clockwise rotation
//        rotate(0.3, 90);
//        //Go straight to the box
//        int countsAfterRotation = inchesToCounts(distanceAfterRotation, true);
//        boolean finished = goCounts(0.3, countsAfterRotation);
//        sleep(1000);
//        if (!finished && opModeIsActive()) {
//            forwardAndRotate(true);
//        }
//    }

    public void forwardAndRotate(boolean clockwise) {

        long startTime = (new Date()).getTime();
        double clockwiseSpeed = clockwise ? 0.7 : -0.7;
        wheels.powerMotors(0.1, -0.8, clockwiseSpeed);
        while(getTime() - startTime < 400) {
            idle();
        }
        wheels.powerMotors(0,0,0);


    }

    public static long getTime() {
        return (new Date()).getTime();
    }

//    1. Lower hand, use relicRotate.setPosition(RELIC_ROTATE_DOWN)
//    2. Open hand, use relicGrab.setPosition(RELIC_GRAB_RELEASE)
//    3. Extend arm, use relicArm, go 1040 counts
//    4. Close hand, use relicGrab.setPosition(RELIC_GRAB_HOLD)
//    5. Retract arm, use relicArm, go back 1040 counts/or until touch button is pressed
//    6. Raise hand, use relicRotate.setPosition(RELIC_ROTATE_UP)

    public void grabRelic() {
        relicRotate.setPosition(RELIC_ROTATE_DOWN);
        relicGrab.setPosition(RELIC_GRAB_RELEASE);

        int extendcounts = isBlue ? 1040 : 1040;

        //extending arm to relic
        moveRelicArm(1, extendcounts);

        relicGrab.setPosition(RELIC_GRAB_HOLD);
        sleep(800);

        //retract relic arm, halfway there rotates relic upward
        moveRelicArm(-1, extendcounts);
    }

    public void moveRelicArm(double armPower, int counts) {
        boolean relicScrewMoving = false;
        boolean armMoving = false;
        boolean up = false;
        int startPosArm = relicArm.getCurrentPosition();
        int startPosScrew = relicScrew.getCurrentPosition();
        // if we are retracting arm, we also want to move the arm up
        if (armPower < 0) {
            relicScrew.setPower(1);
            relicScrewMoving = true;
        }
        relicArm.setPower(armPower);
        armMoving = true;
        while (opModeIsActive() && (armMoving || relicScrewMoving)) {

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

                // arm should stop moving up if it reached the delivery position
                int armscrewup = isBlue ? ARM_SCREW_UP - ARM_SCREW_UP_BLUE_INIT : ARM_SCREW_UP;
                if (relicScrewMoving && Math.abs(relicScrew.getCurrentPosition() - startPosScrew) >= armscrewup) {
                    relicScrew.setPower(0);
                    relicScrewMoving = false;
                }
            }

            //when desired arm position is reached, the arm should stop moving
            if (armMoving && Math.abs(relicArm.getCurrentPosition() - startPosArm) >= counts) {
                relicArm.setPower(0);
                armMoving = false;
            }

            idle();
        }

        //precaution to make sure relicArm and relicScrew all stop
        relicArm.setPower(0);
        relicScrew.setPower(0);
    }

}
