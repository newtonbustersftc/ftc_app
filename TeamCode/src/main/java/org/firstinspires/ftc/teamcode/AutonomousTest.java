package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by NBTeam on 10/27/2017.
 */
@Autonomous(name="AutonomousTest", group ="Main")
public class AutonomousTest extends LinearOpMode {
    MecanumWheels mecanumWheels;
    @Override
    public void runOpMode() throws InterruptedException {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry);
        mecanumWheels.powerMotors(0, 0, 0);
        waitForStart();
        mecanumWheels.goCounts(3000);
        while  (opModeIsActive()) {
            idle();
        }
    }
}
