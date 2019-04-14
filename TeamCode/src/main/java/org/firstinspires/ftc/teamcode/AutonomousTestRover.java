package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Locale;

@Autonomous(name = "TestAuto", group = "Main")
public class AutonomousTestRover extends AutonomousRover {

    @Override
    public void doRunOpMode() throws InterruptedException {
        preRun();
        waitForStart();

        if(!opModeIsActive()){
            return;
        }

        TEST=true;


        //rotateToHeadingTest();

        //dropMarker();

//        arcTest();


//        steerTest();

//        gyroDriveTest();
          rangeDriveTest();

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
          Lights.disableRed();

    }

    private void rotateToHeadingTest() {
        out.append("current, target, cw angle \n");

        double targetHeading = 90.0;

        wheels.powerMotors(0, 0, 0.3);
        while(opModeIsActive()){

            double currentHeading = getGyroAngles().firstAngle;

            double angleToRotate = Heading.clockwiseRotateAngle(new Heading(currentHeading), new Heading(targetHeading));
            telemetry.clear();
            telemetry.addData("heading", currentHeading);
            telemetry.addData("target heading", targetHeading);
            telemetry.addData("cw angle to rotate", angleToRotate);
            telemetry.update();
            out.append(String.format(Locale.US, "%.1f,%.1f,%.1f,\n",
                    currentHeading, targetHeading, angleToRotate));
            sleep(500);
        }
    }

    private void distanceTest() throws InterruptedException {
        wheels.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER); //doing by speed
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int counts = 1000;

        while (counts < 5000) {
            goCounts(0.5, counts);
            sleep(5000);
            goCounts( -0.5, counts);
            sleep(5000);
            counts=counts+1000;
        }
    }

    private void steerTest() {
        logPrefix = "steer";
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
        logPrefix = "gyro";
        double startPower = 0.65;
        double endPower = 0.2;
        double heading = 0;
        double inches = 70;
        double [] kpArr = {0.9, 1.0, 1.1};
        for(double kp : kpArr) {
            clockwiseK = kp;
            moveWithErrorCorrection(startPower, endPower, inches, new GyroErrorHandler(heading));
            sleep(5000);
            moveWithErrorCorrection(-startPower, -endPower, inches, new GyroErrorHandler(heading));
            sleep(5000);
        }
    }

    void rangeDriveTest() throws InterruptedException {
        logPrefix = "range";
        double startPower = 0.65;
        double endPower = 0.2;
        double inches = 60;
        double rangeInInches = 5;
        logPrefix = "rangeTest";
        TEST = true;
        out = new StringBuffer();

        sleep(5000);
        double [] kpArr = {0.012, 0.012, 0.012};

        for(double kp : kpArr) {
            if (!opModeIsActive()) return;
            telemetry.addData("kP", kp);
            telemetry.update();
            out.append(String.format("# kp = %.3f \n", kp));
            TEST = true;

            Heading hcurrent = new Heading(getGyroAngles().firstAngle);
            Heading htarget = new Heading(0);

            double angleToRotate = Heading.clockwiseRotateAngle(hcurrent, htarget);
            rotate(angleToRotate>0, Math.abs(angleToRotate));

            RangeErrorHandler handler1 = new RangeErrorHandler(rangeSensorBackRight,
                    rangeSensorFrontRight, rangeInInches,  true,0);
            handler1.setKP(kp);
            moveWithErrorCorrection(-startPower, -endPower, inches, handler1);
            sleep(5000);

            hcurrent = new Heading(getGyroAngles().firstAngle);
            htarget = new Heading(0);

            angleToRotate = Heading.clockwiseRotateAngle(hcurrent, htarget);
            rotate(angleToRotate>0, Math.abs(angleToRotate));

            RangeErrorHandler handler2 = new RangeErrorHandler(rangeSensorFrontRight,
                    rangeSensorBackRight, rangeInInches,
                    false, 0);
            handler2.setKP(kp);
            moveWithErrorCorrection(startPower, endPower, inches, handler2);
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