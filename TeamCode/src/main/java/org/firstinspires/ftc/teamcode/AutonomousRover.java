package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Date;
import java.util.Locale;

@Autonomous(name = "AutoRoverCrater", group = "Main")
public class AutonomousRover extends LinearOpMode {

    static final double POS_MARKER_FORWARD = 0.8;
    static final double POS_MARKER_UP = 0.45;
    static final double POS_MARKER_BACK = 0.2;

    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor liftMotor;

    Servo hookServo;
    Servo markerServo;

    PixyCam pixyCam;

    private BNO055IMU imu; // gyro
    private Orientation angles; // angles from gyro

    protected boolean depotSide() {
        return false;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        preRun();

        waitForStart();
        landing();

        rotateAndMoveGold();

        sleep(2000);
    }

    public void preRun() {
        motorLeft = hardwareMap.dcMotor.get("wheelsLeft");
        motorRight = hardwareMap.dcMotor.get("wheelsRight");


        pixyCam = hardwareMap.get(PixyCam.class, "pixy");

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
            telemetry.update();
            idle();
        }
    }

    public void landing() throws InterruptedException {
        if(!opModeIsActive()) return;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(-1);
        liftMotor.setTargetPosition(-DriverRover.LATCHING_POS);
        sleep(6000);
        liftMotor.setPower(0);
        sleep(200);

        hookServo = hardwareMap.servo.get("hookServo");
        hookServo.setPosition(0.01);
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
        if(!opModeIsActive()) return;
        boolean goldOnSide = false; //if gold block isn't in the center
        boolean goldOnRight = false;
        PixyCam.Block goldBlock = getGoldBlock();
        telemetry.addData("Gold", goldBlock.toString());
        telemetry.update();
        sleep(2000);

        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Lift Position", liftMotor.getCurrentPosition());

        if (goldBlock!= null && (goldBlock.x > 140 || goldBlock.x < 110)) {
            goldOnSide = true;
            if (goldBlock.x > 140) {
                goldOnRight = true;
                rotate(.3, 30);
            } else {
                rotate(-.3, 30);
            }
        }
        sleep(500);

        int distanceToCenterGold = inchesToCounts(19); //distance is in counts
        int distanceToSideGold = inchesToCounts(21); //distance is in counts

        int extraDistance = 0; //extraDistance is for the depot side only

        if (goldOnSide) {
            if(depotSide()){
                extraDistance = inchesToCounts(8);
            }
            goCounts(0.5, distanceToSideGold + extraDistance);
        } else {
            if(depotSide()){
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
                    rotate(-0.3,Math.abs(currentAngle));
                    goCounts(0.5, inchesToCounts(13));
                    rotate(-0.3, 20);
                    goCounts(0.5, inchesToCounts(8));
                } else {
                    double currentAngle = getGyroAngles().firstAngle;
                    //return to previous heading
                    rotate(0.3,Math.abs(currentAngle));
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
            if(goldOnSide) {
                if(goldOnRight) {
                    rotate(-0.3,20);
                } else {
                    rotate(0.3, 10);
                }
                goCounts(0.5,inchesToCounts(5));
            }
            markerServo.setPosition(POS_MARKER_FORWARD);
            sleep(2000);
        }
    }

    PixyCam.Block getGoldBlock() {
        ElapsedTime time = new ElapsedTime();
        time.reset();
        PixyCam.Block block = null;
        while (block == null ||
                (block.x <= 0 || block.x >= 255) ||
                (block.y <= 0 || block.y >= 255) ||
                (block.width < 8 || block.width > 70) ||
                (block.height < 8 || block.height > 70)
                ) {
            if(!opModeIsActive() || time.seconds() > 5){
                block = null;
                break;
            }
            block = pixyCam.getBiggestBlock(2); // Signature 2 = yellow block
            telemetry.addData("Signature 2",block.toString());
            telemetry.update();
            sleep(20);
        }
        return block;
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
     * rotates robot the given angle, give negative power for counterclockwise rotation and
     * positive power for clockwise rotation
     *
     * @param power
     * @param angle
     */
    void rotate(double power, double angle) {
        if(!opModeIsActive()) return;
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
        if(!opModeIsActive()) return false;

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

    /**
     * initializes gyro
     */
    private void gyroInit() {
        // see the calibration sample opmode;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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
     *
     * @param degrees angle in degrees
     * @return a string
     */
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}