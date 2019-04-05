package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

class Lights {
    private final static int NUM_LIGHTS = 5;
    //red is on digital port
    private static DigitalChannel red;
    private static Servo[] lights = new Servo[NUM_LIGHTS-1];
    private final static int GREEN = 0;
    private final static int BLUE = 1;
    private final static int YELLOW = 2;
    private final static int WHITE = 3;

    static LynxModule lynxModule;

    static void setUpLights(HardwareMap hardwareMap) {
        lynxModule = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        red  = hardwareMap.get(DigitalChannel.class, "light1");
        red.setMode(DigitalChannel.Mode.OUTPUT); //write 1 for off, 0 for on
        red.setState(true);
        for(int i = 2;i <= NUM_LIGHTS; i++) {
            int lightIndex = i - 2;
            lights[lightIndex] = hardwareMap.servo.get("light" + i);
            DriverRover.setUpServo(lights[lightIndex], 0.5, 1);
        }
        checkForGlobalWarnings();
    }

    static void resetLights() {
        red.setState(true);
        for(int i = 2;i <= NUM_LIGHTS; i++) {
            int lightIndex = i - 2;
            lights[lightIndex].setPosition(0);
        };
    }

    static void red(boolean isOn) {red.setState(!isOn);
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
            red(true);
        }
    }

    static boolean hasGlobalWarnings() {
        return lynxModule != null && lynxModule.getGlobalWarnings().size()>0;
    }
}