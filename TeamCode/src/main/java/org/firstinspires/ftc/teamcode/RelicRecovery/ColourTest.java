package org.firstinspires.ftc.teamcode.RelicRecovery;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by NBTeam on 12/15/2017.
 */
//@Autonomous(name = "ColourTest", group = "Main")
public class ColourTest extends LinearOpMode{
    ColorSensor rightColour;
    ColorSensor leftColour;
    DistanceSensor rightDistance;
    DistanceSensor leftDistance;
    @Override
    public void runOpMode() throws InterruptedException {
        rightColour = hardwareMap.get(ColorSensor.class, "Right_Colour");
        leftColour =  hardwareMap.get(ColorSensor.class, "Left_Colour");
        rightDistance = hardwareMap.get(DistanceSensor.class, "Right_Colour");
        leftDistance = hardwareMap.get(DistanceSensor.class, "Left_Colour");


        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        while(!this.isStarted() && !this.isStopRequested()){
            telemetry.clear();
            telemetry(telemetry, relativeLayout, rightColour, rightDistance, "Right");
            telemetry(telemetry, relativeLayout, leftColour, leftDistance, "Left");
            telemetry.update();
            idle();
        }
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

    }

    public static void telemetry(Telemetry telemetry, final View relativeLayout, ColorSensor sensorColor, DistanceSensor sensorDistance, String name){
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        final float[] hsvValues = {0f, 0f, 0f};
        final double SCALE_FACTOR = 255;
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData(name + " Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData(name + " R/G/B/Alpha", sensorColor.alpha()+"/"+sensorColor.red()+"/"+sensorColor.green()+"/"+sensorColor.blue());
        telemetry.addData(name + " Hue", hsvValues[0]);
        if(name.equals("Right")){
            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, hsvValues));
                }
            });
        }



    }
}
