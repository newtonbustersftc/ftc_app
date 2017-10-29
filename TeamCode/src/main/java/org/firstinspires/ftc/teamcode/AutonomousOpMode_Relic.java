package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.setUpServo;

/**
 * Created by Brandon on 10/22/2017.
 */

@Autonomous(name="AutonomousOpMode", group ="Main")
public class AutonomousOpMode_Relic extends LinearOpMode {

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
    public static final double JEWEL_KICK_CENTER =  DriverOpMode_Relic.JEWEL_KICK_CENTER;

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    public void autonomousStart() {
        leftHand = hardwareMap.servo.get("Left-Hand");
        setUpServo(leftHand, LEFT_HAND_IN_POS, LEFT_HAND_OUT_POS);
        rightHand = hardwareMap.servo.get("Right-Hand");
        setUpServo(rightHand, RIGHT_HAND_IN_POS, RIGHT_HAND_OUT_POS);
        jewelArm = hardwareMap.servo.get("Jewel-Arm");
        jewelArm.setPosition(JEWEL_ARM_HOME);
        jewelKick = hardwareMap.servo.get("Jewel-Kick");
        jewelKick.setPosition(JEWEL_KICK_RIGHT);
    }
    public void moveArm(double endPos)
    {
        double currentPos = jewelArm.getPosition();
        int c = 1;
        if(endPos < currentPos)
        {
            c = -1;
        }
        while (Math.abs(endPos - currentPos) >= 0.025 && opModeIsActive())
        {
            currentPos = currentPos + c*0.025;
            jewelArm.setPosition(currentPos);
            telemetry.addData("Arm Position: ", currentPos);
            telemetry.update();
            sleep(40);
        }
    }
    @Override public void runOpMode() {

        relicTemplate = vuforiaInitialize();
        sensorColor = hardwareMap.get(ColorSensor.class, "Color-Sensor");

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        autonomousStart(); //Initialize the servos
        jewelKick.setPosition(JEWEL_KICK_CENTER);
        moveArm(JEWEL_ARM_DOWN);
        int colour = jewelColorCheck();
        //Assuming here that we are the blue alliance
        if(colour == 1) //Colour 1 is red
        {
            jewelKick.setPosition(JEWEL_KICK_RIGHT);
        }
        else if(colour == -1)
        {
            jewelKick.setPosition(JEWEL_KICK_LEFT);
        }
        sleep(1000);
        jewelKick.setPosition(JEWEL_KICK_CENTER);
        sleep(1000);
        moveArm(JEWEL_ARM_HOME);
        sleep(2000);
        jewelKick.setPosition(JEWEL_KICK_RIGHT);
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

    /**
     * returns the ball colour that is on the right
     * @return 1 if red, -1 if blue, and 0 if cannot be determined
     */
    private int jewelColorCheck(){
        // Check for determinable color
        long startTime = System.currentTimeMillis();
        int color = 0;
        while (color == 0 && opModeIsActive()){
            if (sensorColor.blue() > sensorColor.red()){
                color = -1;
            }else if (sensorColor.red() > sensorColor.blue()){
                color = 1;
            }
            if (System.currentTimeMillis() - startTime > 10*1000){ // 10 second cutoff
                break;
            }
        }
        // Return color
        return color;
    }
}
