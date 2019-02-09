package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import static java.lang.System.currentTimeMillis;


public abstract class BaseAutonomous extends LinearOpMode {

    boolean TEST = false;

    protected StringBuffer out = null;
    private BNO055IMU imu; // gyro
    Orientation angles; // angles from gyro

    String logPrefix = "lastrun"; //prefix for a log file

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            doRunOpMode();
        } finally {
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
    }

    abstract public void doRunOpMode() throws InterruptedException;

    /**
     * initializes gyro
     */
    boolean gyroInit() {
        String fileName = "AdafruitIMUCalibration.json";
        // see the calibration sample opmode;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = fileName;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        boolean success = imu.initialize(parameters);
        long startMs = currentTimeMillis();
        while(!imu.isGyroCalibrated() && currentTimeMillis() - startMs < 7000){
            idle();
        }
        if(imu.isGyroCalibrated()){
            BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
            File file = AppUtil.getInstance().getSettingsFile(fileName);
            ReadWriteFile.writeFile(file, calibrationData.serialize());
        }
        return success;
    }
    boolean isGyroError(){
        BNO055IMU.SystemError error = imu.getSystemError();
        if (error != BNO055IMU.SystemError.NO_ERROR) {
            if (out == null) { out = new StringBuffer();}
            out.append(String.format(Locale.US, "# BNO055IMU.SystemError %s\n", error));
            return true;
        } else{
            return false;
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
     * error for java detection
     *
     * @param heading desired heading
     * @return the error (difference between desired heading and actual heading)
     */
    double getRawHeadingError(double heading) {
        return (getGyroAngles().firstAngle - heading);
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


    /**
     * set break or float behavior
     *
     * @param behavior
     */
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        throw new UnsupportedOperationException("setZeroPowerBehavior is not implemented");
    }

    /**
     * @return the selected wheel encoder count
     */
    int getWheelPosition() {
        throw new UnsupportedOperationException("getWheelPosition is not implemented");
    }


    int inchesToCounts(double inches, boolean foward) {
        throw new UnsupportedOperationException("inchesToCounts is not implemented");
    }

    /**
     * @param motorPower power to go in straight line from -1 to 1
     * @param steerPower power to make adjustments clockwise
     */
    void steer(double motorPower, double steerPower) {
        throw new UnsupportedOperationException("steer is not implemented");
    }


    /**
     * moves robot given number of inches using the error handler to keep it straight
     * @param startPower  starting forward power
     * @param errorHandler object that has getError method
     * @param inches      distance forward
     * @param endPower    ending forward power
     * @return true if distance required was travelled, false otherwise
     */
    boolean moveWithErrorCorrection(double startPower, double endPower, double inches, ErrorHandler errorHandler) {

        if (!opModeIsActive()) return false;

        boolean forward = startPower > 0;

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int initialcount = getWheelPosition();
        double steerSpeed; // counterclockwise speed

        double inchesForGradient = 10;
        double slope = (endPower - startPower) / inchesToCounts(Math.min(inchesForGradient, inches), forward);
        double countsForGradient = (inches < inchesForGradient) ?
                0 : Math.abs(inchesToCounts(inches - inchesForGradient, forward));

        double countsSinceStart = Math.abs(getWheelPosition() - initialcount);
        double motorPower = startPower;
        long startTime = currentTimeMillis();
        while (opModeIsActive() && currentTimeMillis() - startTime < 5000 &&
                countsSinceStart < inchesToCounts(inches, forward)) {
            // steer CCW - negative, CW - positive
            steerSpeed = errorHandler.getSteerSpeed();

            if (countsSinceStart > countsForGradient) {
                motorPower = slope * (countsSinceStart - inchesToCounts(inches, forward)) + endPower;
            }
            steer(motorPower, steerSpeed);
            countsSinceStart = Math.abs(getWheelPosition() - initialcount);

        }
        steer(0, 0);

        return opModeIsActive();
    }

    private void logRangeCorrectionHeader() {
        if (out == null) {
            out = new StringBuffer();
        }
        out.append(String.format(Locale.US, "Time,Error,Steer,Type,Range1,Range2\n"));
    }

    private void logRangeCorrection(long timeMs, double error, double steerSpeed, double range1, double range2) {
        if (out == null) {
            out = new StringBuffer();
        }
        out.append(String.format(Locale.US, "%d,%.2f,%.2f,range,%.2f,%.2f\n", timeMs, error, steerSpeed, range1, range2));
    }

    private void logGyroCorrection(long timeMs, double error, double steerSpeed) {
        if (out == null) {
            out = new StringBuffer();
        }
        out.append(String.format(Locale.US, "%d,%.2f,%.2f,gyro\n", timeMs, error, steerSpeed));
    }

    public class GyroErrorHandler implements ErrorHandler {

        private double heading;
        private long startTime;

        GyroErrorHandler(double heading) {
            this.heading = heading;
            startTime = currentTimeMillis();
        }

        @Override
        public double getSteerSpeed() {
            if (isGyroError()) return 0;
            //clockwise = positive steer speed
            double error = getError();
            double steerSpeed = error * getKP();
            if (TEST) logGyroCorrection(currentTimeMillis()-startTime, error, steerSpeed);
            return steerSpeed;
        }

        double getError() {
            double error = getRawHeadingError(heading);
            // consider 1 degree error negligible
            // we need no-correction area to avoid over-correction
            return (Math.abs(error) < 1) ? 0 : error;
        }


        double getKP() {
            //experimental coefficient for proportional correction of the direction
            return 0.01;
        }
    }

    public class RangeErrorHandler implements ErrorHandler {

        private DistanceSensor s1;
        private DistanceSensor s2;
        private double rangeInInches;
        private int errorSign;
        private GyroErrorHandler gyroErrorHandler;
        private long startTime;

        private double KP = 0.02;

        /**
         * @param s1                  leading sensor
         * @param s2                  following sensor
         * @param rangeInInches       how many inches we want to be from the wall
         * @param clockwiseIfTooClose should we rotate clockwise if we are too close?
         * @param gyroHeading         heading for fallback gyro error correction if distance sensors fail
         */
        RangeErrorHandler(DistanceSensor s1,
                          DistanceSensor s2,
                          double rangeInInches,
                          boolean clockwiseIfTooClose,
                          double gyroHeading) {
            this.s1 = s1;
            this.s2 = s2;
            this.rangeInInches = rangeInInches;
            errorSign = clockwiseIfTooClose ? 1 : -1;

            startTime = currentTimeMillis();
            gyroErrorHandler = new GyroErrorHandler(gyroHeading);
            gyroErrorHandler.startTime = startTime;
            if (TEST) logRangeCorrectionHeader();
        }

        @Override
        //clockwise = positive steer speed
        public double getSteerSpeed() {
            double range1 = s1.getDistance(DistanceUnit.INCH);
            double range2 = s2.getDistance(DistanceUnit.INCH);
            double error, steerSpeed;
            if (rangeInBounds(range1) && rangeInBounds(range2)) {
                //if ranges are reasonable, use 2 sensors
                error = getError(range1, range2);
                steerSpeed = error * getKP();
                if (TEST) logRangeCorrection(currentTimeMillis()-startTime, error, steerSpeed, range1, range2);
                return steerSpeed;
            } else {
                if (out == null) { out = new StringBuffer();}
                out.append(String.format(Locale.US, "# Bad range s1: %.2f s2: %.2f\n", range1, range2));
                //if ranges are unreasonable, use gyro sensor only
                return gyroErrorHandler.getSteerSpeed();
            }

        }

        /**
         * @param range1 - distance from the leading sensor (the front sensor when the robot is moving)
         * @param range2 – distance from the following sensor (back sensor)
         * @return error, positive if we are closer to the wall than we want to be
         */
        public double getError(double range1, double range2) {
            /*
            with the sensor mounted on the left side of the robot,
            we want the robot to move clockwise (positive error) if the leading sensor is too close to the wall
            and counterclockwise (negative error) if it is too far from the wall
             */
            double error1 = (rangeInInches - range1) * errorSign;

            /*
            we want the robot to be parallel to the wall
            error2 is how parallel it is to the wall
             */
            double error2 = (range2 - range1) * errorSign;
            return error1 + error2;
        }


        double getKP() {
            //experimental coefficient for proportional correction of the direction
            return KP;
        }

        void setKP(double kp) {
            this.KP = kp;
        }

        /**
         * returns true if range is reasonable, false if it is not
         *
         * @param rangeInInches
         * @return
         */
        boolean rangeInBounds(double rangeInInches) {
            return rangeInInches > 0 && rangeInInches < 50;
        }
    }

}
