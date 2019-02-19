package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

class Lights {
    private final static int NUM_LIGHTS = 5;
    private static Servo[] lights = new Servo[NUM_LIGHTS];
    private final static int RED = 0;
    private final static int GREEN = 1;
    private final static int BLUE = 2;
    private final static int YELLOW = 3;
    private final static int WHITE = 4;

    static LynxModule lynxModule;

    static void setUpLights(HardwareMap hardwareMap) {
        lynxModule = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        for(int i = 1;i <= NUM_LIGHTS; i++) {
            int lightIndex = i - 1;
            lights[lightIndex] = hardwareMap.servo.get("light" + i);
            DriverRover.setUpServo(lights[lightIndex], 0.5, 1);
        }
        checkForGlobalWarnings();
    }

    static void resetLights() {
        for(int i = 1;i <= NUM_LIGHTS; i++) {
            int lightIndex = i - 1;
            lights[lightIndex].setPosition(0);
        };
    }

    static void red(boolean isOn) {
        lights[RED].setPosition(isOn?1:0);
    }

    static void green(boolean isOn) {
        lights[GREEN].setPosition(isOn?1:0);
    }

    static void blue(boolean isOn) {
        lights[BLUE].setPosition(isOn?1:0);
    }

    static void yellow(boolean isOn) {
        lights[YELLOW].setPosition(isOn?1:0);
    }

    static void white(boolean isOn) {
        lights[WHITE].setPosition(isOn?1:0);
    }

    static void checkForGlobalWarnings() {
        if (hasGlobalWarnings()) {
            lights[RED].setPosition(0.6);
        }
    }

    static boolean hasGlobalWarnings() {
        return lynxModule != null && lynxModule.getGlobalWarnings().size()>0;
    }
}