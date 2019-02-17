package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;


@TeleOp(name="Autonomous Options", group="Main")
public class AutonomousOptionsRover extends OpMode {

    enum State  {DisplayAll, DisplaySingle}

    private State menuState = State.DisplayAll;

    private SharedPreferences prefs;
    private SharedPreferences.Editor editor;

    private int selectionIdx = 0;

    // ADD preference names here
    public static final String AUTO_MODE_PREF = "autoMode";
    public static final String DELAY_PREF = "delay";
    public static final String CRATER_MODE_PREF = "shortCraterMode";
    private static final String NONE = "none";

    enum CraterModes {
        LONG, SHORT
    }

    enum AutoMode {
        DEPOT, CRATER
    }

    // ADD preference values here
    // for example:
    private static final String[] AUTO_MODES = {AutoMode.DEPOT.toString(), AutoMode.CRATER.toString()};
    private static final String[] DELAYS = {"0 sec", "1 sec", "2 sec", "3 sec", "4 sec", "5 sec"};
    private static final String[] CRATER_MODES = {CraterModes.LONG.toString(), CraterModes.SHORT.toString()};

    public static SharedPreferences getSharedPrefs(HardwareMap hardwareMap) {
        return hardwareMap.appContext.getSharedPreferences("autonomous", 0);
    }


    private static Map<String, String[]> prefMap = new HashMap<>();
    static {
        // ADD entries to preference map here
        // for example:
        prefMap.put(DELAY_PREF, DELAYS);
        prefMap.put(CRATER_MODE_PREF, CRATER_MODES);

    }
    private static String[] prefKeys = prefMap.keySet().toArray(new String[prefMap.keySet().size()]);
    static {
        Arrays.sort(prefKeys);
    }
    private static int keyIdx = 0;


    private int getIndex(String val, String[] array) {
        if(array!=null){
            for (int i = 0; i < array.length; i++) {
                if (array[i].equals(val)) {
                    return i;
                }
            }
        }
        return -1;
    }

    @Override
    public void init() {
        prefs = getSharedPrefs(hardwareMap);

        editor = prefs.edit();
        for (String key : prefs.getAll().keySet()) {
            telemetry.addData(key, prefs.getString(key, NONE));
        }
    }


    @Override
    public void loop() {
        switch (menuState) {
            case DisplayAll:
                displayAll();
                break;
            case DisplaySingle:
                displaySingle();
                break;
        }
    }

    private void displayAll () {

        telemetry.clear();
        telemetry.addData("Choose", "X - accept Y - change");
        for (String key : prefKeys) {
            telemetry.addData(key, prefs.getString(key, NONE));
        }
        if (gamepad1.y) {
            while (gamepad1.y) {
            }
            menuState = State.DisplaySingle;
        }
    }

    private void displaySingle () {

        telemetry.clear();
        telemetry.addData("Choose", "X - accept Y - change");
        String key = prefKeys[keyIdx];
        String[] array = prefMap.get(key);
        if (key != null) {
            String prefValue = prefs.getString(key, NONE);
            selectionIdx = getIndex(prefValue, array);
            telemetry.addData(key, prefValue);
        }
        if (gamepad1.x) {
            while (gamepad1.x) {
            }
            int nextKeyIdx = keyIdx+1;
            if (nextKeyIdx >= prefKeys.length) {
                keyIdx = 0;
                menuState = State.DisplayAll;
            }
            else {
                keyIdx = nextKeyIdx;
            }
        }
        if (gamepad1.y) {
            while (gamepad1.y) {
            }
            selectionIdx++;
            if (selectionIdx >= array.length) {
                selectionIdx = 0;
            }
            editor.putString(key, array[selectionIdx]);
            editor.apply();
        }
    }
}

