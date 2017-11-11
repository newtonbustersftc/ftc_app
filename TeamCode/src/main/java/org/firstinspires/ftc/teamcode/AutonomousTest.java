package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

import static java.lang.Float.parseFloat;

/**
 * Created by NBTeam on 10/27/2017.
 */
@Autonomous(name = "AutonomousTest", group = "Main")
public class AutonomousTest extends AutonomousOpMode_Relic {

    @Override
    public void runOpMode() throws InterruptedException {
        preRun();
        waitForStart();

        autonomousStart(); //Initialize the servos

        //colorTest();
        gyroTest();

    }

    // gyroTest(): figures out how long the gyro takes to update
    private void gyroTest(){
        setAngles();
        StringBuffer out = new StringBuffer();
        try{
            // turn clockwise, then counter alternating, 3 times each
            for (int n = 0; n < 5; n++){
                // turn clockwise
                out.append("Clockwise 180!\n");
                setAngles();
                wheels.powerMotors(0,0,0.3);
                float initialheading = parseFloat(formatAngle(angles.angleUnit, angles.firstAngle));
                float currentheading = parseFloat(formatAngle(angles.angleUnit, angles.firstAngle));
                while (Math.abs(currentheading - initialheading) < 90 && opModeIsActive()){
                    telemetry.addData("hd", currentheading - initialheading);
                    telemetry.update();
                    out.append(new SimpleDateFormat("HH:mm:ss.SSS").format(new Date()))
                            .append(",")
                            // heading/roll/pitch
                            .append(formatAngle(angles.angleUnit, angles.firstAngle) + "/" + formatAngle(angles.angleUnit, angles.secondAngle) + "/" + formatAngle(angles.angleUnit, angles.thirdAngle))
                            .append("\n");
                    idle();
                    setAngles();
                    currentheading = parseFloat(formatAngle(angles.angleUnit, angles.firstAngle));
                }

                // turn clockwise
                out.append("Clockwise 180!\n");
                wheels.powerMotors(0,0,-0.3);
                setAngles();
                initialheading = parseFloat(formatAngle(angles.angleUnit, angles.firstAngle));
                currentheading = parseFloat(formatAngle(angles.angleUnit, angles.firstAngle));
                while (Math.abs(currentheading - initialheading) < 90 && opModeIsActive()){
                    telemetry.addData("hd", currentheading - initialheading);
                    telemetry.update();
                    out.append(new SimpleDateFormat("HH:mm:ss.SSS").format(new Date()))
                            .append(",")
                            // heading/roll/pitch
                            .append(formatAngle(angles.angleUnit, angles.firstAngle) + "/" + formatAngle(angles.angleUnit, angles.secondAngle) + "/" + formatAngle(angles.angleUnit, angles.thirdAngle))
                            .append("\n");
                    idle();
                    setAngles();
                    currentheading = parseFloat(formatAngle(angles.angleUnit, angles.firstAngle));
                }
            }
        }finally{
            try {
                String name = "gyrotest";
                //log file without the time stamp to find it easier
                File file;
                file = new File(Environment.getExternalStorageDirectory().getPath() +
                        "/FIRST/" + name + ".txt");
                //saving the log file into a file
                OutputStreamWriter outputStreamWriter =
                        new OutputStreamWriter(new FileOutputStream(file));
                outputStreamWriter.write(out.toString());
                outputStreamWriter.close();
                telemetry.addData("File", file.getAbsolutePath());
            } catch (Exception e){
                telemetry.addData("Exception", "File write failed: " + e.toString());
            }
        }
    }

    private void colorTest() {
        jewelKick.setPosition(JEWEL_KICK_CENTER);
        moveArm(JEWEL_ARM_DOWN);
        StringBuffer out = new StringBuffer(); //String Buffer
        try {
            for (int n = 0; n < 5; n++) {
                for (int i = 0; i < 300; i++) {
                    out.append("\n")
                            .append(new SimpleDateFormat("HH:mm:ss.SSS").format(new Date()))
                            .append(",")
                            .append(sensorColor.red() + "/" + sensorColor.green() + "/" + sensorColor.blue());
                    idle();
                }
                moveArm(JEWEL_ARM_HOME);
                sleep(2000);
                moveArm(JEWEL_ARM_DOWN);
                sleep(2000);

                out.append("\n\n\n\n\n");
            }
        } finally {
            try {
                String name = "colortest";
                //log file without the time stamp to find it easier
                File file;
                file = new File(Environment.getExternalStorageDirectory().getPath() +
                        "/FIRST/" + name + ".txt");
                //saving the log file into a file
                OutputStreamWriter outputStreamWriter =
                        new OutputStreamWriter(new FileOutputStream(file));
                outputStreamWriter.write(out.toString());
                outputStreamWriter.close();
                telemetry.addData("File", file.getAbsolutePath());
            } catch (Exception e) {
                telemetry.addData("Exception", "File write failed: " + e.toString());
            }
            telemetry.update();
        }
    }
}
