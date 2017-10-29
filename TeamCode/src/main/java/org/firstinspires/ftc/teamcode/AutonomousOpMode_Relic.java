package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Brandon on 10/22/2017.
 */

@Autonomous(name="AutonomousOpMode", group ="Main")
public class AutonomousOpMode_Relic extends LinearOpMode {

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    @Override public void runOpMode() {

        relicTemplate = vuforiaInitialize();

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate(); // Starts looking for VuMarks

        RelicRecoveryVuMark vuMark = findVuMark();

        while (opModeIsActive()){

        }

    }

    private VuforiaTrackable vuforiaInitialize(){
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

    private RelicRecoveryVuMark findVuMark (){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        long vuStartTime = System.currentTimeMillis();
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && opModeIsActive()){
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark", "not visible");
            telemetry.update();
            // If the seeing takes too long (10 seconds here), then just go with putting a block in the left one.
            if (System.currentTimeMillis() - vuStartTime > 10*1000){
                vuMark = RelicRecoveryVuMark.LEFT;
            }
        }
        telemetry.addData("VuMark", "%s visible", vuMark);
        telemetry.update();
        return vuMark;
    }
}
