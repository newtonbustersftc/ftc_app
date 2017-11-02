package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by NBTeam on 10/27/2017.
 */
@Autonomous(name="AutonomousTest", group ="Main")
public class AutonomousTest extends AutonomousOpMode_Relic {

    @Override
    public void runOpMode() throws InterruptedException {
        preRun();
        waitForStart();
        goCounts(0.4, 3000);
        while  (opModeIsActive()) {
            idle();
        }
    }
}
