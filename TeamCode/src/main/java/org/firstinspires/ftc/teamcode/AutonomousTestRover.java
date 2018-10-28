package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Date;

@Autonomous(name = "AutoTestRover", group = "Main")
public class AutonomousTestRover extends AutonomousRover {

    @Override
    public void runOpMode() throws InterruptedException {
        preRun();
        waitForStart();
        //distanceTest();
//        rotate(0.12,90);
//        rotate(-0.12, 180);
//        rotate(0.12, 180);
        landing();
    }

    private void distanceTest() throws InterruptedException {
        int counts = 1000;
        while (counts < 5000) {
            goCounts(-0.3, counts);
            sleep(5000);
            goCounts( -0.3, counts);
            sleep(5000);
            counts=counts+1000;
        }

    }
}
