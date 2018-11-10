package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

@TeleOp(name = "DriverRover", group = "Main")
public class DriverRover extends OpMode {
    //TODO: EDIT LATER ON
    private static final double MINPOWER = 0.1;

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor liftMotor;

    private ServoImplEx hookServo;
    private Servo markerServo;

    //for now, we don't support robot reversing the forward direction
    private final boolean forward = true;

    boolean hookReleased = true; //when true, the hook is released
    boolean hookButtonPressed = false; //when true, the hook button is pressed down

    //optimal lift height encoder count is -11600 to -12800 so -12200 is optimal
    public static final int LATCHING_POS = 12200;
    static final int LATCHING_POS_LOW = 11600;
    static final int LATCHING_POS_HIGH = 12800;

    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("wheelsLeft");
        motorRight = hardwareMap.dcMotor.get("wheelsRight");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        // run by power
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reset the encoder for the lift motor
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // float zero power
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {
        //ServoImplEx allows to energize and deenergize servo
        // we don't want to hook servo to keep position when robot is lifting or lowering
        hookServo = (ServoImplEx)hardwareMap.servo.get("hookServo");
        //set position to 0 for releasing the hook and use position 1 to close hook
        setUpServo(hookServo, AutonomousRover.POS_HOOK_OPEN, AutonomousRover.POS_HOOK_CLOSED);

        markerServo = hardwareMap.servo.get("markerServo");
        //for moving the arm forward, use 1, for moving it back, use 0
        setUpServo(markerServo, AutonomousRover.POS_MARKER_BACK, AutonomousRover.POS_MARKER_FORWARD);
    }

    @Override
    public void loop() {
        //power given to the left side wheels, power given to the right side wheels
        //forward as a direction
        double leftForward;
        double rightForward;
        double liftMotorPower;

        int sign = forward ? 1 : -1;
        if (!gamepad1.y) {
            //Arcade Drive
            rightForward = -(scaled(gamepad1.left_stick_y) + sign * scaled(gamepad1.left_stick_x));
            leftForward = -(scaled(gamepad1.left_stick_y) - sign * scaled(gamepad1.left_stick_x));
        } else {
            //Tank Drive
            leftForward = -scaled(gamepad1.left_stick_y);
            rightForward = -scaled(gamepad1.right_stick_y);
        }

        if ((gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right)) {
            // dPadDrive: Forward, Backwards, in place CClockwise and Clockwise
            //We are using robot coordinates
            double dpadSpeed = 0.2;
            if (gamepad1.dpad_up) {
                rightForward = dpadSpeed;
                leftForward = dpadSpeed;
            } else if (gamepad1.dpad_down) {
                rightForward = -dpadSpeed;
                leftForward = -dpadSpeed;
            } else if (gamepad1.dpad_left) {
                //rotating ccw
                rightForward = sign * dpadSpeed;
                leftForward = -sign * dpadSpeed;
            } else {
                //rotating cw
                leftForward = sign * dpadSpeed;
                rightForward = -sign * dpadSpeed;
            }
        }

        // Lift motor when given negative power, the lift rises and lowers when given positive power
        if (gamepad1.x || gamepad1.a) {
            int currentPos = Math.abs(liftMotor.getCurrentPosition());
            if (gamepad1.x) {
                if (LATCHING_POS < currentPos) {
                    liftMotorPower = 0;
                } else {
                    liftMotorPower = -1;
                }
            } else {
                if (currentPos < 1000) {
                    liftMotorPower = 0.3;
                } else {
                    if (currentPos < LATCHING_POS_LOW) {
                        deenergizeHook();
                    }
                    liftMotorPower = 1;
                }
            }
        } else {
            liftMotorPower = 0.0;
        }

        if (gamepad1.b) {
            hookButtonPressed = true;
        } else {
            if (hookButtonPressed) {
                energizeHook();
                if (hookReleased) {
                    hookServo.setPosition(1);
                } else {
                    hookServo.setPosition(0);
                }
                hookReleased = !hookReleased;
            }
            hookButtonPressed = false;
        }

        //driving backwards
        if (!forward) { //when start, front direction is the intake side, lightStrip2
            leftForward = -leftForward;
            rightForward = -rightForward;
        }

        //todo adjust the deadband
        if ((rightForward > -0.01) && (rightForward < 0.01))
            rightForward = 0;
        else if ((rightForward > -MINPOWER) && (rightForward < MINPOWER))
            rightForward = MINPOWER * rightForward / Math.abs(rightForward);

        if ((leftForward > -0.01) && (leftForward < 0.01))
            leftForward = 0;
        else if ((leftForward > -MINPOWER) && (leftForward < MINPOWER))
            leftForward = MINPOWER * leftForward / Math.abs(leftForward);

        rightForward = Range.clip(rightForward, -1, 1);
        leftForward = Range.clip(leftForward, -1, 1);

        telemetry.addData("left", leftForward);
        telemetry.addData("right", rightForward);
        telemetry.addData("lift", liftMotor.getCurrentPosition());

        powerMotors(rightForward, leftForward, liftMotorPower);
    }

    private void powerMotors(double rightForward, double leftForward, double liftMotorPower) {
        motorLeft.setPower(leftForward);
        motorRight.setPower(rightForward);
        liftMotor.setPower(liftMotorPower);
    }

    /**
     * deenergize hook servo (if enabled)
     */
    void deenergizeHook() {
        if (hookServo.isPwmEnabled()) {
            hookServo.setPwmDisable();
        }
    }

    /**
     * energize hook servo (if disabled)
     */
    void energizeHook() {
        if (!hookServo.isPwmEnabled()) {
            hookServo.setPwmEnable();
        }
    }

    private double scaled(double x) {
        return (x / 1.07) * (.62 * x * x + .45);
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

}