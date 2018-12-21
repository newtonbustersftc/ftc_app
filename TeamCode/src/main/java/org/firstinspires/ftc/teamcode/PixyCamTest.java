package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


//@TeleOp(name = "PixyCamTest")
public class PixyCamTest extends OpMode {

    PixyCam pixyCam;
    PixyCam.Block block1;
    PixyCam.Block block2;
    ElapsedTime elapsedTime = new ElapsedTime();

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init()
    {
        pixyCam = hardwareMap.get(PixyCam.class, "pixy");
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop()
    {
        if (elapsedTime.milliseconds() > 200) // Update every tenth of a second.
        {
            elapsedTime.reset();
//            block1 = pixyCam.getBiggestBlock(1);
//            telemetry.addData("Signature 1", block1.toString());
            block2 = pixyCam.getBiggestBlock(2);
            telemetry.addData("Signature 2", block2.toString());
        }
    }
}
