package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.RelicDelivery.ExtendingToRotate;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.RelicDelivery.Lowering;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.RelicDelivery.Retracting;
import static org.firstinspires.ftc.teamcode.DriverOpMode_Relic.RelicDelivery.Wall;

/**
 * Created by JASMINE on 10/22/17.
 * This class has the code for our driver controlled mode.
 */
@TeleOp(name = "DriverOpMode", group = "Main")
public class DriverOpMode_Relic extends OpMode {


    double DPAD_POWER = 0.2;

    private MecanumWheels mecanumWheels; //Controls mecanum wheel motors.
    private boolean backButtonPressed; //True when the button for changing direction is pressed.

    private DcMotor pusher; //The motor controling the glyph pusher.
    private DcMotor relicArm; //The motor that extends the relic arm.
    private DcMotor relicScrew; //The motor that raises/lowers the relic arm.

    private DcMotor lift; //DcMotor for the lift

    //5 discrete lift levels: on the ground, slightly above the ground, the second, third, and fourth cryptobox levels.
    int[] LIFT_LEVEL_COUNTS = {0, 300, 2200, 4100, 6000}; //The lift motor counts for the discrete lift levels.
    private int LIFT_COUNT_MAX = LIFT_LEVEL_COUNTS[4]; // Maximum lift height.
    private int LIFT_COUNTS_TOLERANCE = 100; // The allowed tolerance above and below the desired level.


    boolean rightBumperPressed = false; // True when pressed. Raises the discrete lift level by one level.
    boolean leftBumperPressed = false; // True when pressed. Lowers the discrete lift level by one level.
    double targetLiftLevel = 0; //Lift level can be 0, 1, 2, 3, and 4 or something in between.

    /*
        True when the lift touch button isn't pressed.
        The touch button at the lowest lift position is like a safety for the lift.
        When pressed, the lift should stop.
     */
    private boolean liftTouchReleased;
    /*
    True when the relic arm touch button isn't pressed.
    The touch button at the most retracted point of the relic arm.
    When pressed, the relic arm should stop retracting.
     */
    private boolean relicTouchReleased;

    private DigitalChannel relicTouchSensor; //Touch sensor at farthest back position on the relic arm
    private DigitalChannel liftTouchSensor; //Touch sensor at lowest position on the lift
    private DigitalChannel screwTouchSensor; //Touch sensor at lowest position on Relic arm screw

    Servo leftHand; // The servo controlling the left lift grabber.
    public static final double LEFT_HAND_IN_POS = 1.0; // Holding position.
    public static final double LEFT_HAND_OUT_POS = 0.5; // Release position.

    Servo rightHand; // The servo controlling the right lift grabber.
    public static final double RIGHT_HAND_IN_POS = 0.0; // Holding position.
    public static final double RIGHT_HAND_OUT_POS = 0.5; // Release position.

    Servo jewelArm; // The servo controlling the jewel arm with a color sensor and kicker at the end.
    public static final double JEWEL_ARM_HOME = 0.72; // Initial position
    public static final double JEWEL_ARM_DOWN = 0.02; // Down position
    public static final double JEWEL_ARM_VERTICAL = 0.55; // Up position

    Servo jewelKick; // The servo controlling the jewel kicker.
    public static final double JEWEL_KICK_RIGHT = 0.8; // start (rest) position and counterclockwise kick
    public static final double JEWEL_KICK_LEFT = 0.15; // clockwise kick
    public static final double JEWEL_KICK_CENTER = 0.47; // The middle position.

    Servo relicRotate; // The servo that rotates the relic grabber up and down.
    public static final double RELIC_ROTATE_UP = 0; //holding the relic above the arm
    public static final double RELIC_ROTATE_DOWN = 0.6; //holding the relic in place to grab or put down

    Servo relicGrab; // The servo controlling the relic hand grabber.
    public static final double RELIC_GRAB_HOLD = 0.25; //holding the relic
    public static final double RELIC_GRAB_RELEASE = 0.6; //letting go of the relic

    //difference in counts between lowest and delivery (highest) arm positions
    public static final int ARM_SCREW_UP = 2400;

    //the height of relic screw to clear the wall with the relic
    public static final int ARM_SCREW_LOADED_UP = 2900;

    //the height of relic screw to place relic in the far zone
    public static final int ARM_SCREW_PLACE_RELIC = 2300;

    //the length of the relic arm to place relic in far zone
    public static final int RELIC_ARM_PLACE_RELIC = 2300;

    //safe length to rotate relic grabber up or down
    public static final int RELIC_ARM_ROTATE = 1900;

    // The relic delivery states.
    enum RelicDelivery {
        Wall,
        ExtendingToRotate,
        ExtendingForPlacement,
        Lowering,
        Release,
        RetractingToRotate,
        Retracting
    }

    private RelicDelivery relicDeliveryState = Wall; // The current state of the relic delivery.

    //Need to allow time for the servos to reach their positions.
    public long relicGrabStartTime = 0; // The timer for the grabber.
    public long relicRotateStartTime = 0; // The timer for the relic rotation.

    @Override
    public void init() {
        //Initializing motors and sensors.
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry);
        mecanumWheels.powerMotors(0, 0, 0);
        pusher = hardwareMap.dcMotor.get("Pusher");
        relicArm = hardwareMap.dcMotor.get("Relic Arm");
        relicScrew = hardwareMap.dcMotor.get("Relic Screw");
        relicTouchSensor = hardwareMap.digitalChannel.get("Touch-Sensor Relic");
        relicTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        relicTouchReleased = relicTouchSensor.getState();
        // When the relic arm touch button is pressed, we want the motor encoder counts to be 0.
        if (!relicTouchReleased) {
            resetEncoders(relicArm, true);
        }
        screwTouchSensor = hardwareMap.digitalChannel.get("Touch-Screw");
        screwTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        boolean screwTouchReleased = screwTouchSensor.getState();
        // We want the lowest arm position to have encoder counts of 0.
        if (!screwTouchReleased) {
            resetEncoders(relicScrew, true);
        }
        relicArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicArm.setPower(0);
        relicDeliveryState = RelicDelivery.Wall;

        relicScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicScrew.setPower(0);

        lift = hardwareMap.dcMotor.get("Lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE); // Positive power brings the lift up and negative power brings it down.
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // We brake when no power is being applied.
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
        // Initializes the position of all of the servos after the start button is pressed
        // so the robot doesn't move during intitialization.
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

        controlPush(); // Glyph pusher controls.
        controlLift(); // Glyph lift controls.
        controlGrip(); // Glyph grabber controls.
        controlRelic(); // Controls all motors and servos for the relic.
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
        // fine control with up and down overrides coarse control left and right
        // up - clockwise, down - counterclockwise
        if (Math.abs(gamepad1.right_stick_y) > 0.4) clockwise = (-gamepad1.right_stick_y/Math.abs(gamepad1.right_stick_y)) * MecanumWheels.MIN_CLOCKWISE;

        //We are using robot coordinates
        //D-pad is used for slow speed movements.
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            double forward = 0;
            double right = 0;
            if (gamepad1.dpad_up) {
                forward = DPAD_POWER;
            } else if (gamepad1.dpad_down) {
                forward = -DPAD_POWER;
            }
            if (gamepad1.dpad_right) {
                right = DPAD_POWER * 1.5;
            } else if (gamepad1.dpad_left) {
                right = -DPAD_POWER * 1.5;
            }
            mecanumWheels.powerMotors(forward, right, clockwise);
        } else {
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            mecanumWheels.powerMotors(forward, right, clockwise);
        }
    }

    /**
     * Use second game pad right trigger to push the glyph out.
     * Use second game pad "A" button to retract the pushing plate.
     */
    private void controlPush() {
        if (gamepad2.right_trigger > 0) {
            pusher.setPower(-0.3 * (gamepad2.right_trigger) - 0.4);
        } else if (gamepad2.a) {
            pusher.setPower(0.4);
        } else {
            pusher.setPower(0);
        }
    }

    /**
     * Use first game pad left bumper for automatic glyph delivery.
     * Use second game pad D-pad for relic arm control.
     * Use first game pad triggers to grab and rotate the relic.
     */
    private void controlRelic() {
        if (gamepad1.left_bumper) {
            deliverRelic();
            return;
        }

        boolean screwTouchPressed = !screwTouchSensor.getState();
        if (gamepad2.dpad_up) {
            relicScrew.setPower(1);
        } else if (gamepad2.dpad_down && !screwTouchPressed) {
            relicScrew.setPower(-1);
        } else {
            relicScrew.setPower(0);
        }

        boolean touchPressed = !relicTouchSensor.getState();
        if (gamepad2.dpad_left) {
            relicArm.setPower(0.8);
        } else if (gamepad2.dpad_right && !touchPressed) {
            relicArm.setPower(-0.8);
        } else {
            relicArm.setPower(0);
            relicDeliveryState = RelicDelivery.Wall;
        }

        // code for relic hand
        if (gamepad1.left_trigger > 0) {
            setPercentOpen(relicRotate, gamepad1.left_trigger);
        } else {
            setPercentOpen(relicRotate, 0);
        }
        if (gamepad1.right_trigger > 0) {
            setPercentOpen(relicGrab, gamepad1.right_trigger);
        } else {
            setPercentOpen(relicGrab, 0);
            relicDeliveryState = RelicDelivery.Wall;
        }
    }

    /**
     * Use game pad two left trigger to grab and release the glyph.
     */
    private void controlGrip() {
        float percentOpen = gamepad2.left_trigger;
        setPercentOpen(rightHand, percentOpen);
        setPercentOpen(leftHand, percentOpen);
    }

    /**
     * Use game pad 2 right and left bumpers for discrete lift control.
     * Use game pad 2 "X" and "Y" buttons for continuous glyph height control.
     */
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
        if (gamepad2.right_bumper) {
            if (!rightBumperPressed) {
                rightBumperPressed = true;
                increaseLiftLevel();
            }
        } else {
            rightBumperPressed = false;
        }

        if (gamepad2.left_bumper) {
            if (!leftBumperPressed) {
                leftBumperPressed = true;
                decreaseLiftLevel();
            }
        } else {
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

                if (Math.abs(targetCounts - currentCounts) <= 0.5 * LIFT_COUNTS_TOLERANCE) {
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


    void increaseLiftLevel() {
        targetLiftLevel = Math.floor(targetLiftLevel);
        if (targetLiftLevel < 4) {
            targetLiftLevel++;
        }
    }

    void decreaseLiftLevel() {
        targetLiftLevel = Math.ceil(targetLiftLevel);
        if (targetLiftLevel > 0) {
            targetLiftLevel--;
        }
    }

    int getTargetCounts(int liftLevel) {
        return LIFT_LEVEL_COUNTS[liftLevel];
    }

    void setTargetLevelFromCounts(int counts) {
        if (counts < LIFT_LEVEL_COUNTS[1]) {
            targetLiftLevel = (double) counts / (double) LIFT_LEVEL_COUNTS[1];
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

    public void deliverRelic() {

        switch (relicDeliveryState) {

            case Wall:
                if (relicScrew.getCurrentPosition() < ARM_SCREW_LOADED_UP) {
                    relicScrew.setPower(1);
                } else {
                    relicScrew.setPower(0);
                    relicDeliveryState = ExtendingToRotate;
                    relicArm.setPower(1);
                }
                break;
            case ExtendingToRotate:
                if (relicArm.getCurrentPosition() > RELIC_ARM_ROTATE) {
                    setPercentOpen(relicRotate, 1);
                    relicRotateStartTime = System.currentTimeMillis();
                    relicDeliveryState = RelicDelivery.ExtendingForPlacement;
                }
                break;
            case ExtendingForPlacement:

                if (relicArm.getCurrentPosition() > RELIC_ARM_PLACE_RELIC) {
                    relicArm.setPower(0);

                }
                if (System.currentTimeMillis() - relicRotateStartTime > 800) {
                    relicScrew.setPower(-1);
                    relicDeliveryState = Lowering;
                }
                break;
            case Lowering:
                if (relicScrew.getCurrentPosition() < ARM_SCREW_PLACE_RELIC) {
                    relicScrew.setPower(0);
                    setPercentOpen(relicGrab, 1);
                    relicGrabStartTime = System.currentTimeMillis();
                    relicDeliveryState = RelicDelivery.Release;
                }
                break;
            case Release:
                if (System.currentTimeMillis() - relicGrabStartTime > 800) {
                    relicArm.setPower(-1);
                    relicScrew.setPower(1);
                    relicDeliveryState = RelicDelivery.RetractingToRotate;
                }
                break;
            case RetractingToRotate:
                if (relicScrew.getCurrentPosition() >= ARM_SCREW_UP){
                    relicScrew.setPower(0);
                }
                if (relicArm.getCurrentPosition() < RELIC_ARM_ROTATE) {
                    setPercentOpen(relicRotate, 0);
                    setPercentOpen(relicGrab, 0);
                    relicDeliveryState = Retracting;
                }
                break;
            case Retracting:
                boolean touchPressed = !relicTouchSensor.getState();
                if (touchPressed) {
                    relicArm.setPower(0);
                    relicDeliveryState = RelicDelivery.Wall;
                }
                break;
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