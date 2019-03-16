package org.firstinspires.ftc.teamcode.RelicRecovery;

import android.app.Activity;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumWheels;

import static org.firstinspires.ftc.teamcode.RelicRecovery.AutonomousOptions.ALLIANCE_PREF;
import static org.firstinspires.ftc.teamcode.RelicRecovery.AutonomousOptions.START_POSITION_PREF;
import static org.firstinspires.ftc.teamcode.MecanumWheels.MIN_CLOCKWISE;
import static org.firstinspires.ftc.teamcode.MecanumWheels.MIN_FORWARD;
import static org.firstinspires.ftc.teamcode.MecanumWheels.MIN_RIGHT;

/**
 * Created by JASMINE on 10/22/17.
 * This class has the code for our driver controlled mode.
 */
//@TeleOp(name = "DriverRelic", group = "Main")
public class DriverOpMode_Relic extends OpMode {


    private double DPAD_POWER = 0.2; // slow move power

    private MecanumWheels mecanumWheels; //Controls mecanum wheel motors.
    private boolean backButtonPressed; //True when the button for changing direction is pressed.

    //current motor speeds
    private double currentForward;
    private double currentRight;
    private double currentClockwise;

    private DcMotor pusher; //The motor controling the glyph pusher.
    private DcMotor relicArm; //The motor that extends the relic arm.
    private DcMotor relicScrew; //The motor that raises/lowers the relic arm.
    private int numDeliveriesCompleted;

    private boolean armMoving = false;
    private boolean screwMoving = false;

    private DcMotor lift; //DcMotor for the lift

    //5 discrete lift levels: on the ground, slightly above the ground, the second, third, and fourth cryptobox levels.
    private int[] LIFT_LEVEL_COUNTS = {0, 300, 2200, 4100, 6000}; //The lift motor counts for the discrete lift levels.
    private int LIFT_COUNT_MAX = LIFT_LEVEL_COUNTS[4]; // Maximum lift height.
    private int LIFT_COUNTS_TOLERANCE = 100; // The allowed tolerance above and below the desired level.


    private boolean rightBumperPressed = false; // True when pressed. Raises the discrete lift level by one level.
    private boolean leftBumperPressed = false; // True when pressed. Lowers the discrete lift level by one level.
    private double targetLiftLevel = 0; //Lift level can be 0, 1, 2, 3, and 4 or something in between.

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

    private Servo leftWheel;
    private Servo rightWheel;

    private Servo leftHand; // The servo controlling the left lift grabber.
    static final double LEFT_HAND_IN_POS = 0.5; // Holding position.
    static final double LEFT_HAND_OUT_POS = 0.2; // Release position.

    private Servo rightHand; // The servo controlling the right lift grabber.
    static final double RIGHT_HAND_IN_POS = 0.5; // Holding position.
    static final double RIGHT_HAND_OUT_POS = 0.8; // Release position.

    private Servo jewelArm; // The servo controlling the jewel arm with a color sensor and kicker at the end.
    static final double JEWEL_ARM_HOME = 0.72; // Initial position
    static final double JEWEL_ARM_DOWN = 0.02; // Down position
    static final double JEWEL_ARM_VERTICAL = 0.55; // Up position

    private Servo jewelKick; // The servo controlling the jewel kicker.
    static final double JEWEL_KICK_RIGHT = 0.8; // start (rest) position and counterclockwise kick
    static final double JEWEL_KICK_LEFT = 0.15; // clockwise kick
    static final double JEWEL_KICK_CENTER = 0.47; // The middle position.

    private Servo relicRotate; // The servo that rotates the relic grabber up and down.
    static final double RELIC_ROTATE_UP = 0.15; //holding the relic above the arm
    static final double RELIC_ROTATE_DOWN = 0.76; //holding the relic in place to grab or put down

    private Servo relicGrab; // The servo controlling the relic hand grabber.
    static final double RELIC_GRAB_HOME = 0.2; // closed
    static final double RELIC_GRAB_HOLD = 0.299; //holding the relic
    static final double RELIC_GRAB_RELEASE = 0.85; //letting go of the relic

    //difference in counts between lowest and delivery (highest) arm positions
    static final int ARM_SCREW_UP = 3200;

    //height of the arm to clear the wall with the arm extended
    static final int ARM_SCREW_CLEAR_WALL = 3200;

    //height of arm when picking up relic at fully extended legth
    private static final int ARM_SCREW_PICKUP = 2050;

    //the height of relic screw to clear the wall with the relic
    private static final int ARM_SCREW_LOADED_UP = 2900;

    //the height of relic screw to place relic in the far zone
    private static final int ARM_SCREW_PLACE_RELIC = 2400;

    //the length of the relic arm to place relic in far zone
    private static final int RELIC_ARM_PLACE_RELIC = 2300;

    //safe length to rotate relic grabber up or down
    private static final int RELIC_ARM_ROTATE = 1800;

    //length of relic arm completely extended
    private static final int RELIC_ARM_LENGTH = 2400;

    // Relic delivery states for automatic relic delivery
    enum RelicDelivery {
        Wall,
        ExtendingToRotate,
        ExtendingForPlacement,
        Release,
        RetractingToRotate,
        Retracting,
        Transition,
        Done
    }

    private RelicDelivery relicDeliveryState = RelicDelivery.Wall; //The current state of the relic delivery.

    // Relic pickup states for semi-automatic pickup
    enum RelicPickup {
        Init,
        ExtendingToRelic,
        Grabbing,
        RetractingToRotate,
        Retracting,
        TransitionToDelivery,
        Done
    }

    private RelicPickup relicPickupState = RelicPickup.Init; //The current state of relic pickup

    //Need to allow time for the servos to reach their positions.
    private long relicGrabStartTime = 0; // The timer for the grabber.
    private long relicRotateStartTime = 0; // The timer for the relic rotation.
    private long releaseStartTime = 0; // The timer for the screw to go up after release.

    // The timer for the robot transition between delivery and pickup
    private long transitionStartTime = 0;

    // State variable to separate manual relic grab control from automatic
    boolean grabTriggerPressed = false;

    //autonomous options - needed because we grab relic autonomously from some positions
    private boolean isBlue; //true if on blue alliance
    private boolean isCornerPos; //true of robot starts on corner platform

    View relativeLayout;

    @Override
    public void init() {
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);

        String allianceString = prefs.getString(ALLIANCE_PREF, "");
        isBlue = allianceString.equals("blue");

        String startPosString = prefs.getString(START_POSITION_PREF, "");
        isCornerPos = startPosString.equals("corner");

        numDeliveriesCompleted = 0;

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
        relicArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        resetStartTime();
        backButtonPressed = false;
        // Initializes the position of all of the servos after the start button is pressed
        // so the robot doesn't move during intitialization.
        leftHand = hardwareMap.servo.get("Left-Hand");
        setUpServo(leftHand, LEFT_HAND_IN_POS, LEFT_HAND_OUT_POS);
        rightHand = hardwareMap.servo.get("Right-Hand");
        setUpServo(rightHand, RIGHT_HAND_IN_POS, RIGHT_HAND_OUT_POS);
        leftWheel = hardwareMap.servo.get("Wheel-Left");
        leftWheel.setPosition(0);
        rightWheel = hardwareMap.servo.get("Wheel-Right");
        rightWheel.setPosition(1);
        jewelArm = hardwareMap.servo.get("Jewel-Arm");
        jewelArm.setPosition(JEWEL_ARM_HOME);
        jewelKick = hardwareMap.servo.get("Jewel-Kick");
        jewelKick.setPosition(JEWEL_KICK_CENTER); // to prevent the relic arm from getting stuck
        relicRotate = hardwareMap.servo.get("Relic-Rotate");
        setUpServo(relicRotate, RELIC_ROTATE_UP, RELIC_ROTATE_DOWN);
        relicGrab = hardwareMap.servo.get("Relic-Grab");
        setUpServo(relicGrab, RELIC_GRAB_HOLD, RELIC_GRAB_RELEASE);
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    }

    @Override
    public void loop() {

        controlPush(); // Glyph pusher controls.
        controlLift(); // Glyph lift controls.
        controlGrip(); // Glyph grabber controls.
        controlRelic(); // Controls all motors and servos for the relic.
        telemetry();

        // DRIVING

        // disable driver's controls in automatic mode
        if ((gamepad1.left_bumper && relicDeliveryState == RelicDelivery.Transition) ||
                (gamepad1.right_bumper && relicPickupState == RelicPickup.TransitionToDelivery)) {
            //skip the wheels controls below when transitioning
            return;
        }

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
        if (Math.abs(gamepad1.right_stick_y) > 0.6)
            clockwise = (-gamepad1.right_stick_y / Math.abs(gamepad1.right_stick_y)) * MIN_CLOCKWISE;

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
            driverPowerWheels(forward, right, clockwise);
        } else {
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            driverPowerWheels(forward, right, clockwise);
        }
    }

    /**
     * Grdually change the speed from current to the requested
     *
     * @param forward   requested forward from -1 to 1
     * @param right     requested right from -1 to 1
     * @param clockwise requested clockwise from -1 to 1
     */
    void driverPowerWheels(double forward, double right, double clockwise) {

        currentForward = calculateNextSpeed(forward, currentForward, MIN_FORWARD);
        currentRight = calculateNextSpeed(right, currentRight, MIN_RIGHT);
        currentClockwise = calculateNextSpeed(clockwise, currentClockwise, MIN_CLOCKWISE);
        mecanumWheels.powerMotors(currentForward, currentRight, currentClockwise);
    }

    private double calculateNextSpeed(double speed, double currentSpeed, double minSpeed) {
        double STEP = 0.2;
        if (Math.abs(speed) < 0.1) {
            // cut power when going to 0
            STEP = 1;
        }

        double nextSpeed;

        if (Math.abs(currentSpeed - speed) <= STEP) {
            nextSpeed = speed;
        } else {
            if (currentSpeed < speed) {
                //speed is increasing
                nextSpeed = currentSpeed + STEP;
                if (nextSpeed > 1)
                    nextSpeed = 1;
                else if (nextSpeed == 0) {
                    nextSpeed = minSpeed;
                } else if (nextSpeed >= -minSpeed && nextSpeed < 0) {
                    nextSpeed = 0;
                }
            } else {
                //speed is decreasing
                nextSpeed = currentSpeed - STEP;
                if (nextSpeed < -1) {
                    nextSpeed = -1;
                } else if (nextSpeed == 0) {
                    nextSpeed = -minSpeed;
                } else if (nextSpeed <= minSpeed && nextSpeed > 0) {
                    nextSpeed = 0;
                }
            }
        }
        return nextSpeed;

    }

    public void stop() {
        if (rightHand != null && leftHand != null) {
            setPercentOpen(rightHand, 1);
            setPercentOpen(leftHand, 1);
            try {
                Thread.sleep(200);
                // Set the panel back to the default color
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.WHITE);
                    }
                });
            } catch (InterruptedException e) {
                //e.printStackTrace();
            }
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
            return; // disable manual controls in automatic mode
        }

        if (gamepad1.right_bumper) {
            pickupRelic();
            return; // disable manual controls in automatic mode
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
        }

        // code for relic hand
        if (relicPickupState == RelicPickup.ExtendingToRelic) {
            setPercentOpen(relicRotate, 1);
            setPercentOpen(relicGrab, 1);
            if (gamepad1.left_trigger > 0.5) {
                relicGrab(true);
                relicPickupState = RelicPickup.Grabbing;
            }
        } else {
            if (gamepad1.left_trigger > 0) {
                setPercentOpen(relicRotate, gamepad1.left_trigger);
            } else {
                setPercentOpen(relicRotate, 0);
            }
            if (gamepad1.right_trigger > 0) {
                grabTriggerPressed = true;
                relicRelease(gamepad1.right_trigger, true); //reset relic pickup state
            } else {
                relicGrab(grabTriggerPressed); //reset relic delivery state if controlled manually
                grabTriggerPressed = false;
            }
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
                } // else let lift do whatever it is doing
            } else {
                lift.setPower(0);
            }
        }

    }


    private void increaseLiftLevel() {
        targetLiftLevel = Math.floor(targetLiftLevel);
        if (targetLiftLevel < 4) {
            targetLiftLevel++;
        }
    }

    private void decreaseLiftLevel() {
        targetLiftLevel = Math.ceil(targetLiftLevel);
        if (targetLiftLevel > 0) {
            targetLiftLevel--;
        }
    }

    private int getTargetCounts(int liftLevel) {
        return LIFT_LEVEL_COUNTS[liftLevel];
    }

    private void setTargetLevelFromCounts(int counts) {
        if (counts < LIFT_LEVEL_COUNTS[1]) {
            targetLiftLevel = (double) counts / (double) LIFT_LEVEL_COUNTS[1];
        } else {
            targetLiftLevel = 1 + 3 * (counts - LIFT_LEVEL_COUNTS[1]) / (double) (LIFT_COUNT_MAX - LIFT_LEVEL_COUNTS[1]);
        }
    }

    /**
     * Command relic grabber to close and optionally reset relic delivery state
     *
     * @param resetState when true reset relic delivery state
     */
    private void relicGrab(boolean resetState) {
        setPercentOpen(relicGrab, 0);
        relicGrabStartTime = System.currentTimeMillis();
        if (resetState) {
            // when grabbing we want to reset relic delivery state
            relicDeliveryState = RelicDelivery.Wall;
        }
    }


    /**
     * Command relic grabber to open and reset relic pickup state
     *
     * @param percentOpen A number from 0 to 1
     * @param resetState  when true reset relic pickup state
     */
    private void relicRelease(double percentOpen, boolean resetState) {
        setPercentOpen(relicGrab, percentOpen);
        relicGrabStartTime = System.currentTimeMillis();
        if (resetState) {
            //when releasing we want to reset relic pickup state
            relicPickupState = RelicPickup.Init;
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
    static void setUpServo(Servo servo, double inPos, double outPos) {
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
    static void setPercentOpen(Servo servo, double percentOpen) {
        servo.setPosition(percentOpen);
    }

    static void resetEncoders(DcMotor motor, boolean wait) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (wait) {
            while (motor.getCurrentPosition() > 20) {

                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    break;
                }
            }
        }
    }

    /**
     * Adjust the position of the relic screw
     *
     * @param targetPos - target position of the relic screw
     */
    private void relicScrewToPosition(int targetPos) {
        boolean touchReleased = screwTouchSensor.getState();
        if (Math.abs(relicScrew.getCurrentPosition() - targetPos) > 50 && touchReleased) {
            if (relicScrew.getCurrentPosition() > targetPos) {
                relicScrew.setPower(-1);
                screwMoving = true;
            } else {
                relicScrew.setPower(1);
                screwMoving = true;
            }
        } else {
            relicScrew.setPower(0);
            screwMoving = false;
        }
    }

    /**
     * Extend or retract arm to the given position
     *
     * @param power     between -1 or 1, extending if positive, retracting if negative
     * @param targetPos target position of the relic arm
     */
    private void relicArmToPosition(double power, int targetPos) {
        if (power > 0) {
            //extending
            if (relicArm.getCurrentPosition() < targetPos) {
                relicArm.setPower(power);
                armMoving = true;
            } else {
                relicArm.setPower(0);
                armMoving = false;
            }
        } else {
            //retracting
            boolean touchReleased = relicTouchSensor.getState();
            if ((relicArm.getCurrentPosition() > targetPos) && touchReleased) {
                relicArm.setPower(power);
                armMoving = true;
            } else {
                relicArm.setPower(0);
                armMoving = false;
            }
        }
    }

    /**
     * returns screw height depending on arm length
     *
     * @param relicArmPos relic arm length in relic arm motor encoder counts
     * @return screw height in relic screw motor encoder counts
     */
    private int pickupScrewPosition(int relicArmPos) {
        //for relic arm 1600, screw height is 1700
        //for arm length 2300, screw height is ARM_SCEW_PEAKUP
        return (int) ((ARM_SCREW_PICKUP-1600) * (relicArmPos - 1600) / 700.0 + 1700);
    }

    /**
     * Semi-automated pickup of the relic:
     * driver can interrupt the arm extension at any time to make the adjustment robot position or
     * direction of the arm. The transition to Grabbing state happens outside this method,
     * when driver presses left trigger. After that he can resume automated pickup, which would
     * retract and raise the arm to wall clearing height, rotate the relic up, and moves the robot
     * to the delivery position (if the first delivery was completed)
     */
    private void pickupRelic() {
        switch (relicPickupState) {

            case Init:
                setPercentOpen(relicRotate, 1); //rotate down
                relicRelease(1, false); //Don't reset relic pickup state
                relicArmToPosition(1, RELIC_ARM_LENGTH);
                relicScrewToPosition(pickupScrewPosition(relicArm.getCurrentPosition()));
                relicPickupState = RelicPickup.ExtendingToRelic;
                break;
            case ExtendingToRelic:
                setPercentOpen(relicRotate, 1); //rotate down
                relicRelease(1, false); //Don't reset relic pickup state
                relicArmToPosition(1, RELIC_ARM_LENGTH);
                relicScrewToPosition(pickupScrewPosition(relicArm.getCurrentPosition()));
                // since we don't know how much the arm should extend
                // let the driver to grab relic manually
                break;
            case Grabbing:
                setPercentOpen(relicRotate, 1); //keep relic rotated down
                if (System.currentTimeMillis() - relicGrabStartTime > 800) {
                    relicArmToPosition(-1, 0);
                    relicScrewToPosition(ARM_SCREW_CLEAR_WALL);
                    relicPickupState = RelicPickup.RetractingToRotate;
                }
                break;
            case RetractingToRotate:
                setPercentOpen(relicRotate, 1); //rotate down
                relicArmToPosition(-1, RELIC_ARM_ROTATE);
                relicScrewToPosition(ARM_SCREW_CLEAR_WALL);
                if (!armMoving) {
                    setPercentOpen(relicRotate, 0); // rotate up
                    if (numDeliveriesCompleted == 1 && !isCornerPos && getRuntime() < 100) {
                        relicPickupState = RelicPickup.TransitionToDelivery;
                        transitionStartTime = System.currentTimeMillis();
                    } else {
                        relicPickupState = RelicPickup.Retracting;
                    }
                }
                break;
            case TransitionToDelivery:
                relicScrewToPosition(ARM_SCREW_CLEAR_WALL);
                if (screwMoving) {
                    transitionStartTime = System.currentTimeMillis();
                } else {
                    if (System.currentTimeMillis() - transitionStartTime < 2000) {
                        if (isBlue) {
                            mecanumWheels.powerMotors(0, 0.6, -0.3, true);
                        } else {
                            mecanumWheels.powerMotors(-0.2, 0, 0.3, true);
                        }
                    } else {
                        mecanumWheels.powerMotors(0, 0, 0);
                        relicPickupState = RelicPickup.Done;
                    }
                }
                break;
            case Retracting:
                relicArmToPosition(-1, 0);
                relicScrewToPosition(ARM_SCREW_CLEAR_WALL);
                if (!armMoving && !screwMoving) {
                    relicPickupState = RelicPickup.Done;
                }
                break;
            case Done:
                break;
        }
    }

    /**
     * Automated relic delivery. extending and retracting of the arm can be stopped at any point.
     * After the first delivery is completed and the conditions are right,
     * it will attempt to transition the robot with the extended arm to the pickup position.
     */
    private void deliverRelic() {


        switch (relicDeliveryState) {
            case Wall:
                if (relicScrew.getCurrentPosition() < ARM_SCREW_LOADED_UP) {
                    relicScrew.setPower(1);
                } else {
                    relicScrew.setPower(0);
                    relicDeliveryState = RelicDelivery.ExtendingToRotate;
                }
                break;
            case ExtendingToRotate:
                relicArmToPosition(1, RELIC_ARM_PLACE_RELIC);
                if (relicArm.getCurrentPosition() > RELIC_ARM_ROTATE) {
                    setPercentOpen(relicRotate, 1);
                    relicRotateStartTime = System.currentTimeMillis();
                    relicDeliveryState = RelicDelivery.ExtendingForPlacement;
                }
                break;
            case ExtendingForPlacement:
                setPercentOpen(relicRotate, 1);
                relicArmToPosition(1, RELIC_ARM_PLACE_RELIC);
                relicScrewToPosition(ARM_SCREW_PLACE_RELIC);
                if (!armMoving && !screwMoving && (System.currentTimeMillis() - relicRotateStartTime) > 1500) {
                    relicRelease(1, true);
                    relicDeliveryState = RelicDelivery.Release;
                }
                break;
            case Release:
                if (System.currentTimeMillis() - relicGrabStartTime > 800) {
                    relicDeliveryState = RelicDelivery.RetractingToRotate;
                    releaseStartTime = System.currentTimeMillis();
                    numDeliveriesCompleted++;
                }
                break;
            case RetractingToRotate:
                if (System.currentTimeMillis()-releaseStartTime>100) {
                    relicScrewToPosition(ARM_SCREW_UP);
                }
                relicArmToPosition(-1, RELIC_ARM_ROTATE);
                if (!armMoving) {
                    setPercentOpen(relicRotate, 0);
                    relicGrab(false); //don't reset relicDeliveryState
                    if (numDeliveriesCompleted == 1 && !isCornerPos) {
                        relicDeliveryState = RelicDelivery.Transition;
                        transitionStartTime = System.currentTimeMillis();
                    } else {
                        relicDeliveryState = RelicDelivery.Retracting;
                    }
                }
                break;
            case Transition:
                relicScrewToPosition(ARM_SCREW_CLEAR_WALL);
                if (screwMoving) {
                    transitionStartTime = System.currentTimeMillis();
                } else {
                    if (System.currentTimeMillis() - transitionStartTime < 2000) {
                        if (isBlue) {
                            mecanumWheels.powerMotors(-0, -0.6, 0.3, true);
                        } else {
                            mecanumWheels.powerMotors(0.2, 0, -0.3, true);
                        }
                    } else {
                        mecanumWheels.powerMotors(0, 0, 0);
                        relicDeliveryState = RelicDelivery.Done;
                    }
                }
                break;
            case Retracting:
                relicArmToPosition(-1, 0);
                relicScrewToPosition(ARM_SCREW_UP);
                if (!armMoving && !screwMoving) {
                    relicDeliveryState = RelicDelivery.Done;
                }
                break;
            case Done:
                break;
        }
    }

    private void telemetry() {
        double runtime = getRuntime();
        String alliance = isBlue ? "BLUE" : "RED";
        telemetry.addData(alliance+" Runtime", (int)runtime);
        telemetry.addData("Pickup State", relicPickupState);
        telemetry.addData("Delivery State", relicDeliveryState);
        telemetry.addData("lift target level", targetLiftLevel);
        telemetry.addData("lift", lift.getCurrentPosition() + " released: " + liftTouchReleased);
        telemetry.addData("relic arm", relicArm.getCurrentPosition() + " moving: " + armMoving + " released: " + relicTouchReleased);
        telemetry.addData("relic screw", relicScrew.getCurrentPosition() + " moving: " + screwMoving);
        telemetry.addData("pusher", gamepad2.right_trigger);
        if (relicRotate != null &&  relicGrab != null){
            telemetry.addData("Relic rotate", gamepad1.left_trigger);
            telemetry.addData("Grab", gamepad1.right_trigger);
        }
        if (runtime > 75) {
            final int color = (runtime > 90 && runtime < 110) ? Color.GREEN : Color.YELLOW;
            // see SensorColor sample for an example of how to change the color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(color);
                }
            });
        }
    }
}