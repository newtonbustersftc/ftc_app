package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;



/**
 * Created by Aryoman on 2/16/2016.
 * revised by JASMINE on 2/17/2016.
 * revised by Simone on 3/24/2016.
 */

@TeleOp(name="Autonomous Options", group="Main")

public class AutonomousOptions extends OpMode {

    enum State  {DisplayAll, DisplaySingle}

    private State menuState = State.DisplayAll;

    private SharedPreferences prefs;
    private SharedPreferences.Editor editor;

    private int selectionIdx = 0;

    // ADD preference names here
    //   public static final String DELAY_PREF = "delay";
    //   public static final String START_TILE_PREF = "startTile";
    private static final String NONE = "none";
    public static final String ALLIANCE_PREF = "alliance";
    public static final String START_POSITION_PREF = "startposition";
    // ADD preference values here
    // for example:
    //   private static final String[] DELAYS = {"0 sec", "3 sec", "5 sec", "7 sec"};
    //   private static final String[] START_POS = {"3rd tile from corner", "4th tile from corner"};
    public static final String[] ALLIANCES = {"blue", "red"};
    public static final String[] START_POSITIONS ={"corner", "other"};


    public static SharedPreferences getSharedPrefs(HardwareMap hardwareMap) {
        return hardwareMap.appContext.getSharedPreferences("autonomous", 0);
    }


    private static Map<String, String[]> prefMap = new HashMap<>();
    static {
        // ADD entries to preference map here
        // for example:
        //    prefMap.put(DELAY_PREF, DELAYS);
        //    prefMap.put(START_TILE_PREF, START_POS);
        prefMap.put(ALLIANCE_PREF, ALLIANCES);
        prefMap.put(START_POSITION_PREF, START_POSITIONS);
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