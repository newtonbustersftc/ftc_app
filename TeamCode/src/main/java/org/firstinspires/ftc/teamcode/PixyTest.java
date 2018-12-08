package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by NBTeam on 12/16/2017, created from Michael Vierra's example
 */


//@Autonomous(name = "PixyTest", group = "Main")
public class PixyTest extends LinearOpMode {
    I2cDeviceSynch pixy; //our Pixy device

    @Override
    public void runOpMode() throws InterruptedException {

        //setting up Pixy to the hardware map
        pixy = hardwareMap.i2cDeviceSynch.get("pixy");

        pixy.setLogging(true);
        pixy.setLoggingTag("PIXY");

        //required to "turn on" the device
        pixy.engage();

        waitForStart();

        while (opModeIsActive()) {
            readPixy(0x51);
            readPixy(0x52);
  //        readPixy(0x50);

            telemetry.update();

            // after running for a while, we always get
            // com.qualcomm.ftcrobotcontroller E/LynxI2cDeviceSynch: exception thrown during lynx communication
            // com.qualcomm.ftcrobotcontroller E/LynxI2cDeviceSynch: com.qualcomm.hardware.lynx.LynxNackException: (DQ15U6X6 #2) LynxI2cReadSingleByteCommand: nack received: I2C_OPERATION_IN_PROGRESS:41
            // the longer sleep, the less probable the exception
            // if less then 100 ms, it happens in a few seconds
            // if more than 800 ms, it rarely happens
            sleep(50);
        }

    }

    private void readPixy(int signatureAddress) {

        // (Re)engage the object with its underlying services.
        // If the object is presently engaged, this method has no effect.
        pixy.engage();

        // read data for given signature
        byte[] pixyData;

        if (signatureAddress == 0x50) {
            // pixy_LEGO_protocol_1.0.pdf
            // general mode: query address 0x50
            // 6 bytes that describe the largest detected block:
            // byte 0,1: Number of blocks that match the specified signature
            // byte 2: X value of center of largest detected block, ranging between 0 and 255. An x value of 255 is the far right­side of Pixy’s image.
            // byte 3: Y value of center of largest detected block, ranging between 0 and 199. A value of 199 is the far bottom­side of Pixy’s image.
            // byte 4: Width of largest block, ranging between 1 and 255. A width of 255 is the full width of Pixy’s image.
            // byte 5: Height of largest block, ranging between 1 and 200. A height of 200 is the full height of Pixy’s image.

            pixyData = pixy.read(signatureAddress, 6);
            int signature = getNumber(pixyData[0], pixyData[1]);
            int x = 0xff & pixyData[2];
            int y = 0xff & pixyData[3];
            int width = 0xff & pixyData[4];
            int height = 0xff & pixyData[5];

            telemetry.addData("Signature", signature);
            telemetry.addData("(x, y) ", "(" + x + "," + y + ")");
            telemetry.addData("(width, height) ", "(" + width + "," + height + ")");

        } else {

            // pixy_LEGO_protocol_1.0.pdf
            // signature query: query address 0x51 thru 0x57
            // 5 bytes that describe the largest detected block for a signature:
            // byte 0: Number of blocks that match the specified signature
            // byte 1: X value of center of largest detected block, ranging between 0 and 255. An x value of 255 is the far right­side of Pixy’s image.
            // byte 2: Y value of center of largest detected block, ranging between 0 and 199. A value of 199 is the far bottom­side of Pixy’s image.
            // byte 3: Width of largest block, ranging between 1 and 255. A width of 255 is the full width of Pixy’s image.
            // byte 4: Height of largest block, ranging between 1 and 200. A height of 200 is the full height of Pixy’s image.

            pixyData = pixy.read(signatureAddress, 5);

            int x = 0xff & pixyData[1];
            int y = 0xff & pixyData[2];
            int width = 0xff & pixyData[3];
            int height = 0xff & pixyData[4];
            int numObjects = 0xff & pixyData[0];

            int sig = signatureAddress - 0x50;

            telemetry.addData("NumObjects" + sig, numObjects);
            telemetry.addData("(x, y) " + sig, "(" + x + "," + y + ")");
            telemetry.addData("(width, height) " + sig, "(" + width + "," + height + ")");
        }


    }

    // All values in the object block are 16-bit words, sent least-signifcant byte first (little endian).
    // So, for example, when sending the sync word 0xaa55, Pixy sends 0x55 (first byte) then 0xaa (second byte).
    private static short getNumber(byte byte1, byte byte2) {
        return (short) (((byte2 & 0xFF) << 8) | (byte1 & 0xFF));
    }

}
