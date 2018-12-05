package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoTestRover", group = "Main")
public class AutonomousTestRover extends AutonomousRover {

    @Override
    public void doRunOpMode() throws InterruptedException {
        preRun();
        waitForStart();

        rangeDriveTest();

//        steerTest();

//        gyroDriveTest();

//        distanceTest();

//        rotate(0.15,90);
//        rotate(-0.12, 180);
//        rotate(0.12, 180);

//        landing();

//        goldPosition = getGoldPosition();
//        telemetry.addData("Gold Position", goldPosition);
//        telemetry.update();
//        sleep(10000);

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

    void rangeDriveTest() throws InterruptedException {
        double startPower = 0.7;
        double endPower = 0.2;
        double inches = 69;
        double rangeInInches = 5;
        logPrefix = "rangeTest";
        TEST = true;
        out = new StringBuffer();

        double [] kpArr = {0.5, 0.1, 0.05, 0.01};

        for(double kp : kpArr) {
            telemetry.addData("kP", kp);
            telemetry.update();
            out.append(String.format("# kp = %.1f \n", kp));
            RangeErrorSource errorSourceBackward = new RangeErrorSource(rangeSensorBackLeft, rangeInInches, false);
            errorSourceBackward.setKP(kp);
            moveWithProportionalCorrection(-startPower, -endPower, inches, errorSourceBackward);
            sleep(5000);

            RangeErrorSource errorSourceForward = new RangeErrorSource(rangeSensorFrontLeft, rangeInInches, true);
            errorSourceForward.setKP(kp);
            moveWithProportionalCorrection(startPower, endPower, inches, errorSourceForward);
            sleep( 5000);
        }
    }
}