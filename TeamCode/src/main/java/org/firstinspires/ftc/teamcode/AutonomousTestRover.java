package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoTestRover", group = "Main")
public class AutonomousTestRover extends AutonomousRover {

    @Override
    public void runOpMode() throws InterruptedException {
        preRun();
        waitForStart();
        distanceDriveTest();
        //distanceTest();
        //steerTest();
        //gyroDriveTest();
//        rotate(0.12,90);
//        rotate(-0.12, 180);
//        rotate(0.12, 180);
//        landing();

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

    private void steerTest() {
        double steerPower = 0.15;
        telemetry.addData("Steer", "forward clockwise");
        telemetry.update();
        steer(0.3, steerPower);
        sleep(2000);
        steer(0, 0);
        sleep(3000);
        telemetry.addData("Steer", "backwards counterclockwise");
        telemetry.update();
        steer(-0.3, -steerPower);
        sleep(2000);
        steer(0,0);
        sleep(3000);

        telemetry.addData("Steer", "forward counterclockwise");
        telemetry.update();
        steer(0.3, -steerPower);
        sleep(2000);
        steer(0, 0);
        sleep(3000);
        telemetry.addData("Steer", "backwards clockwise");
        telemetry.update();
        steer(-0.3, steerPower);
        sleep(2000);
        steer(0,0);
        sleep(3000);
    }

    void gyroDriveTest() throws InterruptedException {
        double startPower = 0.3;
        double endPower = 0.1;
        double heading = 0;
        double inches = 66;
        moveWithProportionalCorrection(startPower, endPower, inches, new GyroErrorSource(heading));

        sleep( 5000);
        moveWithProportionalCorrection(-startPower, -endPower, inches, new GyroErrorSource(heading));
        sleep(5000);
    }

    void distanceDriveTest() throws InterruptedException {
        double startPower = 0.3;
        double endPower = 0.1;
        double inches = 66;
        double rangeInInches = 7;
        ErrorSource errorSourceForward = new RangeErrorSource(rangeSensorLeft, rangeInInches, true);
        ErrorSource errorSourceBackward = new RangeErrorSource(rangeSensorLeft, rangeInInches, false);
        moveWithProportionalCorrection(startPower, endPower, inches, errorSourceForward);

        sleep( 5000);
        moveWithProportionalCorrection(-startPower, -endPower, inches, errorSourceBackward);
        sleep(5000);
    }
}