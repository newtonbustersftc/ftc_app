package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Date;
import java.util.Locale;

@Autonomous(name = "AutoRoverCrater", group = "Main")
public class AutonomousRover extends LinearOpMode {

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
        markerServo.setPosition(0.45);
        liftMotor.setPower(1);
        liftMotor.setTargetPosition(0);
        sleep(6000);
        liftMotor.setPower(0);
        sleep(200);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
        sleep(3000);
        int distanceForward = inchesToCounts(7);
        goCounts(0.3, distanceForward);



    }

    public void rotateAndMoveGold() throws InterruptedException {
        double rotatePower = 0.2;
        boolean goldOnSide = false; //if gold block isn't in the center
        boolean goldOnRight = false;
        PixyCam.Block goldBlock = pixyCam.getBiggestBlock(2); // Signature 2 = yellow block
        telemetry.addData("Gold", goldBlock.toString());
        while (goldBlock.x > 140 || goldBlock.x < 110) {
            goldOnSide = true;
            if (goldBlock.x > 140) {
                goldOnRight = true;
                motorRight.setPower(-rotatePower);
                motorLeft.setPower(rotatePower);
            } else {
                motorRight.setPower(rotatePower);
                motorLeft.setPower(-rotatePower);
            }
            goldBlock = pixyCam.getBiggestBlock(2); // Signature 2 = yellow block
            telemetry.addData("Gold", goldBlock.toString());
            telemetry.update();
        }
        motorRight.setPower(0);
        motorLeft.setPower(0);
        sleep(1000);
        goldBlock = pixyCam.getBiggestBlock(2);
        telemetry.update();
        sleep(5000);

        int distanceToCenterGold = inchesToCounts(17); //distance is in counts
        int distanceToSideGold = inchesToCounts(19); //distance is in counts

        if (goldOnSide) {
            goCounts(0.3, distanceToSideGold);
        } else {
            goCounts(0.3, distanceToCenterGold);
        }

        //if facing crater, we are done
        if (depotSide()) {
            int extraDistanceCenter = inchesToCounts(17); //distance is in counts

            if (!goldOnSide) {
                goCounts(0.3, extraDistanceCenter);
            } else {
                goCounts(0.3, inchesToCounts(5));
                if (goldOnRight) {
                    //TODO: rotate ccw and move to depot
                } else {
                    //TODO: rotate cw and move to depot
                }

            }
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
     * rotates robot the given angle, give negative power for counterclockwise rotation and
     * positive power for clockwise rotation
     *
     * @param power
     * @param angle
     */
    void rotate(double power, double angle) {
        double originalHeading = getGyroAngles().firstAngle;
        double currentHeading = originalHeading;
        telemetry.addData("current heading", originalHeading);
        telemetry.update();
        sleep(3000);
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
        sleep(3000);
        currentHeading = getGyroAngles().firstAngle;
        telemetry.addData("current heading", currentHeading);
        telemetry.update();
        sleep(3000);
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
     * @param degrees angle in degrees
     * @return a string
     */
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}