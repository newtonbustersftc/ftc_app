package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by JASMINE on 10/22/17.
 * This class has the code for our driver controlled mode.
 */
@TeleOp(name = "DriverOpMode", group = "Main")
public class DriverOpMode_Relic extends OpMode {


    double DPAD_POWER = 0.3;

    private MecanumWheels mecanumWheels;
    private boolean backButtonPressed;

    private DcMotor pusher;
    private DcMotor relicArm;
    private DcMotor relicScrew;

    private DcMotor lift; //DcMotor for the lift


    int[] LIFT_LEVEL_COUNTS = {0, 300, 2200, 4100, 6000};
    private int LIFT_COUNT_MAX = LIFT_LEVEL_COUNTS[4];
    private int LIFT_COUNTS_TOLERANCE = 100;


    boolean rightBumperPressed = false;
    boolean leftBumperPressed = false;
    double targetLiftLevel = 0; //Lift level can be 0, 1, 2, 3, and 4 or something inbetween.

    private boolean liftTouchReleased;
    private boolean relicTouchReleased;

    private DigitalChannel relicTouchSensor; //Touch sensor at farthest back position on the relic arm
    private DigitalChannel liftTouchSensor; //Touch sensor at lowest position on the lift

    Servo leftHand;
    public static final double LEFT_HAND_IN_POS = 1.0; //0.63;
    public static final double LEFT_HAND_OUT_POS = 0.5; //0.48;

    Servo rightHand;
    public static final double RIGHT_HAND_IN_POS = 0.0; //0.25;
    public static final double RIGHT_HAND_OUT_POS = 0.5; //0.5;

    Servo jewelArm;
    public static final double JEWEL_ARM_HOME = 0.72; // home position
    public static final double JEWEL_ARM_DOWN = 0.02; // down position
    public static final double JEWEL_ARM_VERTICAL = 0.55; // down position

    Servo jewelKick;
    public static final double JEWEL_KICK_RIGHT = 0.8; // start (rest) position and counterclockwise kick
    public static final double JEWEL_KICK_LEFT = 0.15; // clockwise kick
    public static final double JEWEL_KICK_CENTER = 0.47;

    Servo relicRotate;
    public static final double RELIC_ROTATE_UP = 0; //holding the relic above the arm
    public static final double RELIC_ROTATE_DOWN = 0.6; //holding the relic in place to grab or put down

    Servo relicGrab;
    public static final double RELIC_GRAB_HOLD = 0.25; //holding the relic
    public static final double RELIC_GRAB_RELEASE = 0.6; //letting go of the relic

    //difference in counts between lowest and delivery (highest) arm positions
    public static final int ARM_SCREW_UP = 2400;

    @Override
    public void init() {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry);
        mecanumWheels.powerMotors(0, 0, 0);
        pusher = hardwareMap.dcMotor.get("Pusher");
        relicArm = hardwareMap.dcMotor.get("Relic Arm");
        relicScrew = hardwareMap.dcMotor.get("Relic Screw");
        relicTouchSensor = hardwareMap.digitalChannel.get("Touch-Sensor Relic");
        relicTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        relicTouchReleased = relicTouchSensor.getState();
        if (!relicTouchReleased) {
            resetEncoders(relicArm, true);

        }
        resetEncoders(relicScrew, true);

        relicArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicArm.setPower(0);
        relicScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicScrew.setPower(0);

        lift = hardwareMap.dcMotor.get("Lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftTouchSensor = hardwareMap.digitalChannel.get("Touch-Sensor");
        liftTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        liftTouchReleased = liftTouchSensor.getState();
        if (!liftTouchReleased) {
            resetEncoders(lift, true);

        }
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(0);
        telemetry();
    }

    @Override
    public void start() {
        backButtonPressed = false;
        leftHand = hardwareMap.servo.get("Left-Hand");
        setUpServo(leftHand, LEFT_HAND_IN_POS, LEFT_HAND_OUT_POS);
        rightHand = hardwareMap.servo.get("Right-Hand");
        setUpServo(rightHand, RIGHT_HAND_IN_POS, RIGHT_HAND_OUT_POS);
        jewelArm = hardwareMap.servo.get("Jewel-Arm");
        jewelArm.setPosition(JEWEL_ARM_HOME);
        jewelKick = hardwareMap.servo.get("Jewel-Kick");
        jewelKick.setPosition(JEWEL_KICK_CENTER); // to prevent the relic arm from getting stuck
        relicRotate = hardwareMap.servo.get("Relic-Rotate");
        setUpServo(relicRotate, RELIC_ROTATE_UP, RELIC_ROTATE_DOWN);
        relicGrab = hardwareMap.servo.get("Relic-Grab");
        setUpServo(relicGrab, RELIC_GRAB_HOLD, RELIC_GRAB_RELEASE);
    }

    @Override
    public void loop() {

        controlPush();
        controlLift();
        controlGrip();
        controlRelic();
        telemetry();

        // DRIVING

        //Changing direction from forward to backward and backward to forward
        //Gamepad back button does not work with motorola 3G
        if (gamepad1.y) {
            backButtonPressed = true;
        } else if (backButtonPressed) {
            backButtonPressed = false;
            mecanumWheels.changeDirection();
            //todo: add lights
        }


        double clockwise = gamepad1.right_stick_x;
        //We are using robot coordinates
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            double forward = 0;
            double right = 0;
            if (gamepad1.dpad_up) {
                forward = DPAD_POWER;
            } else if (gamepad1.dpad_down) {
                forward = -DPAD_POWER;
            }
            if (gamepad1.dpad_right) {
                right = DPAD_POWER * 2;
            } else if (gamepad1.dpad_left) {
                right = -DPAD_POWER * 2;
            }
            mecanumWheels.powerMotors(forward, right, clockwise);
        } else {
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            mecanumWheels.powerMotors(forward, right, clockwise);
        }
    }

    private void controlPush() {
        if(gamepad2.right_trigger > 0){
            pusher.setPower(-0.3*(gamepad2.right_trigger)-0.3);
        }
        else if(gamepad2.a){
            pusher.setPower(0.3);
        }
        else{
            pusher.setPower(0);
        }
    }

    private void controlRelic(){
        if(gamepad2.dpad_up){
            relicScrew.setPower(1);
        } else if(gamepad2.dpad_down){
            relicScrew.setPower(-1);
        } else {
            relicScrew.setPower(0);
        }

        boolean touchPressed = !relicTouchSensor.getState();
        if(gamepad2.dpad_left){
            relicArm.setPower(0.8);
        } else if(gamepad2.dpad_right && !touchPressed){
            relicArm.setPower(-0.8);
        } else {
            relicArm.setPower(0);
        }

        // code for relic hand
        if(gamepad1.left_trigger > 0){
            setPercentOpen(relicRotate, gamepad1.left_trigger);
        } else {
            setPercentOpen(relicRotate, 0);
        }
        if(gamepad1.right_trigger > 0){
            setPercentOpen(relicGrab, gamepad1.right_trigger);
        } else {
            setPercentOpen(relicGrab, 0);
        }
    }

    private void controlGrip() {
        float percentOpen = gamepad2.left_trigger;
        setPercentOpen(rightHand, percentOpen);
        setPercentOpen(leftHand, percentOpen);
    }

    private void controlLift() {

        // liftTouchSensor.getState==true means the button is NOT PRESSED
        boolean touchPressed = !liftTouchSensor.getState();
        if (touchPressed) {
            if (liftTouchReleased) {
                resetEncoders(lift, false);
                liftTouchReleased = false;
            } else {
                if (lift.getCurrentPosition() < 50 && !lift.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                    lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

        } else {
            liftTouchReleased = true;
        }

        // right bumper increases lift level by 1, left bumper decreases lift level by 1
        if(gamepad2.right_bumper){
            if (!rightBumperPressed) {
                rightBumperPressed = true;
                increaseLiftLevel();
            }
        } else{
            rightBumperPressed = false;
        }

        if (gamepad2.left_bumper){
            if (!leftBumperPressed) {
                leftBumperPressed = true;
                decreaseLiftLevel();
            }
        } else{
            leftBumperPressed = false;
        }

        int liftposition = lift.getCurrentPosition();
        if (gamepad2.y && liftposition < LIFT_COUNT_MAX) {
            lift.setPower(0.6);
            setTargetLevelFromCounts(liftposition);
        } else if (gamepad2.x && !touchPressed) {
            lift.setPower(-0.35);
            setTargetLevelFromCounts(liftposition);
        } else {
            double roundedTargetLiftLevel = Math.round(targetLiftLevel);
            if (roundedTargetLiftLevel == targetLiftLevel) { // discrete lift level control
                int targetCounts = getTargetCounts((int) roundedTargetLiftLevel);
                int currentCounts = lift.getCurrentPosition();

                if(Math.abs(targetCounts - currentCounts) <= 0.5*LIFT_COUNTS_TOLERANCE) {
                    lift.setPower(0);
                } else if (Math.abs(targetCounts - currentCounts) > LIFT_COUNTS_TOLERANCE) {
                    if (targetCounts > currentCounts && currentCounts <= LIFT_COUNT_MAX) {
                        lift.setPower(0.6);
                    } else if (targetCounts < currentCounts && !touchPressed) {
                        lift.setPower(-0.35);
                    } else {
                        lift.setPower(0);
                    }
                } else {
                    // let lift do whatever it is doing
                }
            } else {
                lift.setPower(0);
            }
        }

    }


    void increaseLiftLevel(){
        targetLiftLevel = Math.floor(targetLiftLevel);
        if (targetLiftLevel<4) {
            targetLiftLevel++;
        }
    }
    void decreaseLiftLevel(){
        targetLiftLevel = Math.ceil(targetLiftLevel);
        if (targetLiftLevel>0){
            targetLiftLevel--;
        }
    }
    int getTargetCounts(int liftLevel){
        return LIFT_LEVEL_COUNTS[liftLevel];
    }

    void setTargetLevelFromCounts (int counts) {
        if (counts < LIFT_LEVEL_COUNTS[1]) {
            targetLiftLevel = (double)counts / (double)LIFT_LEVEL_COUNTS[1];
        } else {
            targetLiftLevel = 1 + 3 * (counts - LIFT_LEVEL_COUNTS[1]) / (double) (LIFT_COUNT_MAX - LIFT_LEVEL_COUNTS[1]);
        }
    }

    /**
     * Takes the holding position and release position,
     * Makes them the minimum and maximum servo positions
     * After this method, you can pass percentOpen as the servo position.
     * Holding position would be 0, complete release position would be 1.
     *
     * @param servo  - servo motor
     * @param inPos  - holding position between 0 and 1
     * @param outPos - releasing position between 0 and 1
     */
    public static void setUpServo(Servo servo, double inPos, double outPos) {
        // scale t0 the range between inPos and outPos
        double min = Math.min(inPos, outPos);
        double max = Math.max(inPos, outPos);
        servo.scaleRange(min, max);

        // make sure in position = 0, out position = 1
        if (inPos > outPos) {
            servo.setDirection(Servo.Direction.REVERSE);
        } else {
            servo.setDirection(Servo.Direction.FORWARD);
        }

        setPercentOpen(servo, 0);
    }

    /**
     * Sets percent open
     *
     * @param servo       - servo motor to use
     * @param percentOpen is the number between 0 and 1
     */
    public static void setPercentOpen(Servo servo, double percentOpen) {
        servo.setPosition(percentOpen);
    }

    public static void resetEncoders(DcMotor motor, boolean wait) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (wait) {
            while (motor.getCurrentPosition() > 10) {

                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    break;
                }
            }
        }

    }

    void telemetry() {
        telemetry.addData("pusher", gamepad2.right_trigger);
        telemetry.addData("lift target level", targetLiftLevel);
        telemetry.addData("lift", lift.getCurrentPosition());
        telemetry.addData("touch sensor released", liftTouchReleased);
        telemetry.addData("relic arm", relicArm.getCurrentPosition());
        telemetry.addData("relic sensor released", relicTouchReleased);
        telemetry.addData("relic screw", relicScrew.getCurrentPosition());
    }
}