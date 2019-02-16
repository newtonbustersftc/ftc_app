package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lights {
    final static int NUM_LIGHTS = 5;
    static Servo[] lights = new Servo[NUM_LIGHTS];
    final static int RED = 0;
    final static int GREEN = 1;
    final static int BLUE = 2;
    final static int YELLOW = 3;
    final static int WHITE = 4;

    static LynxModule lynxModule;

    public static void setUpLights(HardwareMap hardwareMap) {
        lynxModule = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        for(int i = 1;i <= NUM_LIGHTS; i++) {
            int lightIndex = i - 1;
            lights[lightIndex] = hardwareMap.servo.get("light" + i);
            DriverRover.setUpServo(lights[lightIndex], 0.5, 1);
        }
        checkForGlobalWarnings();
    }

    public static void resetLights() {
        for(int i = 1;i <= NUM_LIGHTS; i++) {
            int lightIndex = i - 1;
            lights[lightIndex].setPosition(0);
        };
    }

    public static void red(boolean isOn) {
        lights[RED].setPosition(isOn?1:0);
    }

    public static void green(boolean isOn) {
        lights[GREEN].setPosition(isOn?1:0);
    }

    public static void blue(boolean isOn) {
        lights[BLUE].setPosition(isOn?1:0);
    }

    public static void yellow(boolean isOn) {
        lights[YELLOW].setPosition(isOn?1:0);
    }

    public static void white(boolean isOn) {
        lights[WHITE].setPosition(isOn?1:0);
    }

    static void checkForGlobalWarnings() {
        if (hasGlobalWarnings()) {
            Lights.red(true);
            Lights.white(true);
        }
    }

    static boolean hasGlobalWarnings() {
        return lynxModule != null && lynxModule.getGlobalWarnings().size()>0;
    }
}