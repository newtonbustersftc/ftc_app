package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Date;

@Autonomous(name = "AutoRoverCrater", group = "Main")
public class AutonomousRover extends BaseAutonomous {

    static final double POS_MARKER_FORWARD = 0.8;
    static final double POS_MARKER_UP = 0.45;
    static final double POS_MARKER_BACK = 0.2;

    static final double POS_HOOK_CLOSED = 0.5;
    static final double POS_HOOK_OPEN = 0.2;

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor liftMotor;

    private Servo hookServo;
    private Servo markerServo;

    private PixyCam pixyCam;

    DistanceSensor rangeSensorLeft;


    protected boolean depotSide() {
        return false;
    }

    @Override
    public void doRunOpMode() throws InterruptedException {
        preRun();

        waitForStart();
        landing();

        rotateAndMoveGold();

        sleep(2000);
    }

    public void preRun() {
        motorLeft = hardwareMap.dcMotor.get("wheelsLeft");
        motorRight = hardwareMap.dcMotor.get("wheelsRight");

        rangeSensorLeft = hardwareMap.get(DistanceSensor.class, "rangeSensor");

        pixyCam = hardwareMap.get(PixyCam.class, "pixy");

        // run by power
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // float zero power
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        gyroInit();

        // make sure gyro is calibrated
        while (!this.isStarted() && !this.isStopRequested()) {
            telemetry.clearAll();
            logGyro(false);
            telemetry.update();
            idle();
        }
    }

    public void landing() throws InterruptedException {
        if (!opModeIsActive()) return;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(-1);
        liftMotor.setTargetPosition(-DriverRover.LATCHING_POS);
        sleep(6000);
        liftMotor.setPower(0);
        sleep(200);

        hookServo = hardwareMap.servo.get("hookServo");
        hookServo.setPosition(POS_HOOK_OPEN);
        sleep(1000);
        markerServo = hardwareMap.servo.get("markerServo");
        markerServo.setPosition(POS_MARKER_BACK);
        liftMotor.setPower(1);
        liftMotor.setTargetPosition(0);
        sleep(1500);
        int distanceForward = inchesToCounts(7);
        goCounts(0.3, distanceForward);
    }

    public void rotateAndMoveGold() throws InterruptedException {
        if (!opModeIsActive()) return;
        boolean goldOnSide = false; //if gold block isn't in the center
        boolean goldOnRight = false;
        PixyCam.Block goldBlock = getGoldBlock();
        if (goldBlock != null) {
            telemetry.addData("Gold", goldBlock.toString());
            telemetry.update();
            log("Detected gold block " + goldBlock.toString());
        }

        sleep(2000);

        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Lift Position", liftMotor.getCurrentPosition());

        if (goldBlock != null && (goldBlock.x > 140 || goldBlock.x < 110)) {
            goldOnSide = true;
            if (goldBlock.x > 140) {
                goldOnRight = true;
                rotate(.3, 30);
            } else {
                rotate(-.3, 30);
            }
        }
        sleep(500);

        int distanceToCenterGold = inchesToCounts(19); //distance is in counts
        int distanceToSideGold = inchesToCounts(21); //distance is in counts

        int extraDistance = 0; //extraDistance is for the depot side only

        if (goldOnSide) {
            if (depotSide()) {
                extraDistance = inchesToCounts(8);
            }
            goCounts(0.5, distanceToSideGold + extraDistance);
        } else {
            if (depotSide()) {
                extraDistance = inchesToCounts(23);
            }
            goCounts(0.5, distanceToCenterGold + extraDistance);
        }

        //if facing crater, we are done
        if (depotSide()) {
            if (goldOnSide) {
                if (goldOnRight) {
                    double currentAngle = getGyroAngles().firstAngle;
                    //return to previous heading
                    rotate(-0.3, Math.abs(currentAngle));
                    goCounts(0.5, inchesToCounts(13));
                    rotate(-0.3, 20);
                    goCounts(0.5, inchesToCounts(8));
                } else {
                    double currentAngle = getGyroAngles().firstAngle;
                    //return to previous heading
                    rotate(0.3, Math.abs(currentAngle));
                    goCounts(0.5, inchesToCounts(13));
                    rotate(0.3, 30);
                    goCounts(0.5, inchesToCounts(8));
                }
            }
            //Deliver team marker.
            markerServo.setPosition(POS_MARKER_FORWARD);
            sleep(2000);
            markerServo.setPosition(POS_MARKER_UP);
            sleep(2000);
        } else {
            if (goldOnSide) {
                if (goldOnRight) {
                    rotate(-0.3, 20);
                } else {
                    rotate(0.3, 10);
                }
                goCounts(0.5, inchesToCounts(5));
            }
            markerServo.setPosition(POS_MARKER_FORWARD);
            sleep(2000);
        }
    }

    PixyCam.Block getGoldBlock() {
        ElapsedTime time = new ElapsedTime();
        time.reset();
        PixyCam.Block block = null;
        while (block == null ||
                (block.x <= 0 || block.x >= 255) ||
                (block.y <= 0 || block.y >= 255) ||
                (block.width < 8 || block.width > 70) ||
                (block.height < 8 || block.height > 70)
                ) {
            if (!opModeIsActive() || time.seconds() > 5) {
                block = null;
                break;
            }
            block = pixyCam.getBiggestBlock(2); // Signature 2 = yellow block
            telemetry.addData("Signature 2", block.toString());
            telemetry.update();
            sleep(100);
        }
        return block;
    }


    /**
     * rotates robot the given angle, give negative power for counterclockwise rotation and
     * positive power for clockwise rotation
     *
     * @param power
     * @param angle
     */
    void rotate(double power, double angle) {
        if (!opModeIsActive()) return;
        double originalHeading = getGyroAngles().firstAngle;
        double currentHeading = originalHeading;
        telemetry.addData("current heading", originalHeading);
        telemetry.update();
        sleep(500);
        long start = new Date().getTime();
        motorRight.setPower(-power);
        motorLeft.setPower(power);
        while (opModeIsActive() && Math.abs(currentHeading - originalHeading) < angle) {
            currentHeading = getGyroAngles().firstAngle;
            //telemetry.addData("current heading", currentHeading);
            //telemetry.update();
        }
        motorRight.setPower(0);
        motorLeft.setPower(0);
        long end = new Date().getTime();
        currentHeading = getGyroAngles().firstAngle;
        telemetry.addData("current heading", currentHeading);
        telemetry.addData("rotate time", end - start);
        telemetry.update();
        sleep(500);
        currentHeading = getGyroAngles().firstAngle;
        telemetry.addData("current heading", currentHeading);
        telemetry.update();
        sleep(500);
    }


    /**
     * moves robot given number of encoder counts
     *
     * @param power  power to apply to all wheel motors
     * @param counts motor encoder counts
     * @return true if the robot travelled all the distance, false otherwise
     * @throws InterruptedException
     */
    boolean goCounts(double power, int counts) throws InterruptedException {
        if (!opModeIsActive()) return false;

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor motor = motorLeft;

        int startPos = motor.getCurrentPosition();
        motorLeft.setPower(power);
        motorRight.setPower(power);

        int currentPos = startPos;
        //Detect robot being stuck: encoder count is not changing for half of a second
        long startTime = (new Date()).getTime();
        long msPassed = 0;
        double previousPos = currentPos;
        boolean finished = true; //this means the robot has traveled all the distance requested
        while (Math.abs(currentPos - startPos) < counts) {
            if (!opModeIsActive()) {
                finished = false;
                break;
            }
            idle();
            currentPos = motor.getCurrentPosition();

            // Stop if the wheel is stuck
            msPassed = (new Date()).getTime() - startTime;
            if (msPassed > 400) {
                if (Math.abs(currentPos - previousPos) < 50) {
                    finished = false;
                    break;

                }
                startTime = (new Date()).getTime();
                previousPos = currentPos;
            }
        }
        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);
        telemetry.addData("Counts Moved", Math.abs(motor.getCurrentPosition() - startPos));
        telemetry.update();
        return finished;
    }

    /**
     * set break or float behavior
     *
     * @param behavior
     */
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motorLeft.setZeroPowerBehavior(behavior);
        motorRight.setZeroPowerBehavior(behavior);
    }

    /**
     * @return the selected wheel encoder count
     */
    int getWheelPosition() {
        return motorLeft.getCurrentPosition();
    }

    /**
     * @param motorPower power to go in straight line from -1 to 1
     * @param steerPower power to make adjustments clockwise
     */
    void steer(double motorPower, double steerPower) {
        double leftPower = motorPower;
        double rightPower = motorPower;
        if (leftPower > 0) {
            if (steerPower > 0)
                leftPower += steerPower;
            else
                rightPower -= steerPower;
        } else {
            if (steerPower > 0) {
                rightPower -= steerPower;
            } else {
                leftPower += steerPower;
            }
        }
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    /**
     * @param inches
     * @param foward true when robot moves foward
     * @return
     */
    int inchesToCounts(double inches, boolean foward) {
        //it doesn't matter whether robot moves foward or backwards
        return inchesToCounts(inches);
    }

    /**
     * This method converts straight distance in inches
     * into left motor encoder counts.
     *
     * @param inches
     * @return
     */
    static int inchesToCounts(double inches) {
        // motorLeft with power 0.3 was used for testing
        return (int) (1000 * inches / 11.5);
    }

    void log(String s) {
        super.log();
        out.append(s);
    }
}