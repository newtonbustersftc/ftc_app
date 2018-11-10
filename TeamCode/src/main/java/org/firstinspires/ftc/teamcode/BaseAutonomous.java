package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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


public abstract class BaseAutonomous extends LinearOpMode {
    protected StringBuffer out = null;
    private BNO055IMU imu; // gyro
    Orientation angles; // angles from gyro

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            doRunOpMode();
        } finally {
            try {
                if(out != null) {
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

                }
            } catch (Exception e) {
                telemetry.addData("Exception", "File write failed: " + e.toString());
            }

        }

    }

    abstract public void doRunOpMode() throws InterruptedException;

    /**
     * initializes gyro
     */
    void gyroInit() {
        // see the calibration sample opmode;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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
     *
     * @param degrees angle in degrees
     * @return a string
     */
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    void log() {
        if(out == null) {
            out = new StringBuffer();
        }
    }
}
