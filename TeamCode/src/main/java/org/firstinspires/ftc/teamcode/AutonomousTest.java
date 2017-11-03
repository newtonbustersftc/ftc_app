package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        jewelKick.setPosition(JEWEL_KICK_CENTER);
        moveArm(JEWEL_ARM_DOWN);

        colorTest();

    }

    private void colorTest() {
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
