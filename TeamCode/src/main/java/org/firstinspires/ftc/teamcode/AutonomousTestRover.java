package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestRover", group = "Main")
public class AutonomousTestRover extends AutonomousRover {

    @Override
    public void doRunOpMode() throws InterruptedException {
        preRun();
        waitForStart();

        if(!opModeIsActive()){
            return;
        }

        TEST=true;
        logPrefix = "rotateTest";

        arcTest();
//        rangeDriveTest();

//        steerTest();

//        gyroDriveTest();

//        distanceTest();
//        for (double power=0.15; power <= .9; power += 0.05) {
//            rotate(power,90);
//            sleep(1000);
//            rotate(-power,90);
//            sleep(1000);
//        }
//
        /**for(double angle = 10; angle <= 90; angle += 10){
            rotate(true, angle);
            sleep(1000);
            rotate(false, angle);
            sleep(1000);
        } **/

//        landing();

//        goldPosition = getGoldPosition();
//        telemetry.addData("Gold Position", goldPosition);
//        telemetry.update();
//        sleep(10000);

    }

    private void distanceTest() throws InterruptedException {
        wheels.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER); //doing by speed
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int counts = 1000;

        while (counts < 5000) {
            goCounts(0.15, counts);
            sleep(5000);
            goCounts( -0.15, counts);
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
        moveWithErrorCorrection(startPower, endPower, inches, new GyroErrorHandler(heading));

        sleep( 5000);
        moveWithErrorCorrection(-startPower, -endPower, inches, new GyroErrorHandler(heading));
        sleep(5000);
    }

    void rangeDriveTest() throws InterruptedException {
        double startPower = 0.7;
        double endPower = 0.2;
        double inches = 70;
        double rangeInInches = 5;
        logPrefix = "rangeTest";
        TEST = true;
        out = new StringBuffer();

        double [] kpArr = {0.02, 0.02, 0.02};

        for(double kp : kpArr) {
            if (!opModeIsActive()) return;
            telemetry.addData("kP", kp);
            telemetry.update();
            out.append(String.format("# kp = %.3f \n", kp));
            TEST = true;
            RangeErrorHandler errorHandlerBackward = new RangeErrorHandler(rangeSensorBackRight,
                    rangeSensorFrontRight, rangeInInches,  true,0);
            errorHandlerBackward.setKP(kp);
            moveWithErrorCorrection(-startPower, -endPower, inches, errorHandlerBackward);
            sleep(5000);

            RangeErrorHandler errorHandlerForward = new RangeErrorHandler(rangeSensorFrontRight,
                    rangeSensorBackRight, rangeInInches,
                    false, 0);
            errorHandlerForward.setKP(kp);
            moveWithErrorCorrection(startPower, endPower, inches, errorHandlerForward);
            sleep( 5000);
        }
    }

    void arcTest() {
        wheels.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER); //doing by speed
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double forward = 0.5;

        for(double clockwise = 0.37; clockwise < 0.45; clockwise += 0.02) {
            if(!opModeIsActive()) return;
            moveHalfCircle(forward, clockwise);
            sleep(10000);
            moveHalfCircle(-forward, -clockwise);
            sleep(10000);
        }
    }

    void moveHalfCircle(double forward, double clockwise) {
        if(!opModeIsActive()) return;

        double originalHeading = getHeading();
        wheels.powerMotors(forward, 0, clockwise);

        double currentHeading = getHeading();
        double gyroDelta = Math.abs(currentHeading - originalHeading);

        while(gyroDelta < 180 && opModeIsActive()) {
            currentHeading = getHeading();
            gyroDelta = Math.abs(currentHeading - originalHeading);
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("GyroDelta", gyroDelta);
            telemetry.update();
            idle();
        }

        wheels.powerMotors(0, 0, 0);
        currentHeading = getHeading();
        telemetry.addData("Heading", currentHeading);
        telemetry.addData("Clockwise was", clockwise);
        telemetry.update();
    }


    double getHeading() {
        double currentHeading = getGyroAngles().firstAngle;

        if(currentHeading > 90) {
            currentHeading -= 360;
        }

        return currentHeading;
    }
}