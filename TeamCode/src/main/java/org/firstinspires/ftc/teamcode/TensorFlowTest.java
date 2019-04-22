/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import static java.lang.System.currentTimeMillis;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
//@Autonomous(name = "TensorFlow", group = "Concept")
//@Disabled
public class TensorFlowTest extends BaseAutonomous {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AbXEDef/////AAAAGU/yaxRV9EUDvGnzyyDBYXcKyDJ9pLb4hlfkcdjSSYUFWen3PVv+3CgFK9M2otdAxlvNvXj2mms/QU9yN/s5sZTgeh+iC39BihOEqC4I+0PYf1L8lqajM0dmEVbsfnGkDcCro+TnDT0vr6zTeJlprz7oNMqGQNZFKSUwEB9cMQR5eLOlUTyt4zhAYvX7Tz2E1cSAXlpVFBzo7/QOqaK7ex/iPo7h1m81KrFDI+yQ6WiveNWAev7kMnau8du//Jyr1I9D6e8QxeL0j/wLDxSrBGAqRnAHcyuQq7Eo39DIseEUu5k849py3hA0uKL8s7NYHg9LxTezwyhSk7cft8aGnCMJ9yMVJEUeTAnE2GEqySWF";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    long startMillis;

    @Override
    public void doRunOpMode()  {
        logPrefix = "tensorflow";

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        try {
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }

            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();
            waitForStart();
            startMillis = currentTimeMillis();

            if (opModeIsActive()) {
                /** Activate Tensor Flow Object Detection. */
                if (tfod != null) {
                    tfod.activate();
                    sleep(1000);
                }

                while (opModeIsActive()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            Recognition goldRecognition = null;
                            int numberOfGolds = 0;
                            int numberOfOthers = 0;
                            double lowestAngle = 90.0;
                            int nr=1;

                            // phone rotation is disabled
                            // the phone is mounted on the side
                            // top (left) is the lowest y coord - 0, bottom (right) is the highest - 800
                            Collections.sort(updatedRecognitions, new Comparator<Recognition>(){
                                @Override
                                public int compare(Recognition r1, Recognition r2) {
                                    return (int)(r1.getBottom()-r2.getBottom());
                                }
                            });

                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition != null) {
                                    telemetry.addData(recognition.getLabel() + (nr++),
                                                    "bottom: " + (int) recognition.getBottom() + "," +
                                                    " conf: " + recognition.getConfidence() + ", (" +
                                                    recognition.getImageWidth() + "," + recognition.getImageHeight()
                                    );
                                }
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    if (recognition.estimateAngleToObject(AngleUnit.DEGREES) < lowestAngle) {
                                        lowestAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                                        goldRecognition = recognition;
                                    }
                                    numberOfGolds++;
                                } else {
                                    numberOfOthers++;
                                }
                            }
                            if (goldRecognition != null) {
                                log(goldRecognition, numberOfGolds, numberOfOthers);
                            }
                            telemetry.addData("File", Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + logPrefix + ".txt");
                            telemetry.addData("numberOfGolds", numberOfGolds);
                            telemetry.addData("numberOfOthers", numberOfOthers);

                            telemetry.addData("Time", (currentTimeMillis() - startMillis) / 1000);
                            telemetry.update();
                            sleep(1000);

//                        Sample code is for when the phone rotation is enabled, and the phone is horizontal
//                        if (updatedRecognitions.size() == 3) {
//                            int goldMineralX = -1;
//                            int silverMineral1X = -1;
//                            int silverMineral2X = -1;
//                            for (Recognition recognition : updatedRecognitions) {
//                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                                    goldMineralX = (int) recognition.getLeft();
//                                } else if (silverMineral1X == -1) {
//                                    silverMineral1X = (int) recognition.getLeft();
//                                } else {
//                                    silverMineral2X = (int) recognition.getLeft();
//                                }
//                            }
//                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Left");
//                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Right");
//                                } else {
//                                    telemetry.addData("Gold Mineral Position", "Center");
//                                }
//                            }
//                        }

                        }
                    }
                }
            }
        } finally {
            if (tfod != null) {
                tfod.deactivate();
                tfod.shutdown();
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * If the number is not one, do not log anything
     * @param r
     * @param numberOfGold
     * @param numberOfOthers
     */
    void log(Recognition r, int numberOfGold, int numberOfOthers) {
        if (out == null) {
            out = new StringBuffer();
            // table header followed by new line
            out.append("Time,Number of Golds,Number of Others,Confidence,Angle,Left,Right,Bottom,Top,\n");
        }
        out.append(currentTimeMillis()-startMillis).append(",");
        out.append(numberOfGold).append(",");
        out.append(numberOfOthers).append(",");

        if(numberOfGold > 0) {
            out.append(r.getConfidence()).append(",");
            out.append(r.estimateAngleToObject(AngleUnit.DEGREES)).append(",");
            out.append(r.getLeft()).append(",");
            out.append(r.getRight()).append(",");
            out.append(r.getBottom()).append(",");
            out.append(r.getTop()).append(",");
        } else {
            out.append(",,,,,,");
        }
        out.append("\n"); // new line
    }
}
