package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * MotorTestJoystickTeleOp
 *
 * Diagnostic TeleOp: all four motors run together, driven by a single
 * joystick's magnitude - no directional mixing or vector math. Full
 * deflection in ANY direction (up, down, left, right, diagonal) sends
 * the same full-power value to all four motors at once. This is purely
 * for confirming every motor spins correctly across a range of power
 * levels, not for actual directional driving.
 *
 * Controls:
 *   left_stick_y / left_stick_x -> whichever has the larger magnitude
 *                                  sets the power level for all 4 motors
 *
 * Configure four motors in your robot configuration with these names:
 *   "FR_Drive", "BR_Drive", "FL_Drive", "BL_Drive"
 *
 * Also reads two analog position encoders (plugged into analog ports)
 * configured with these names:
 *   "FR_Position", "BR_Position"
 *
 * Motor power is capped at MAX_SPEED, and each motor is monitored
 * against a current (amperage) limit - if a motor draws more than
 * CURRENT_LIMIT_AMPS, its power is cut until it drops back under
 * the limit. Note: the REV hub's absolute hardware current limit
 * (9.2A per port) can't be changed from code - this is a *software*
 * limit layered on top, set lower than that hardware ceiling so it
 * actually takes effect first.
 *
 * Two continuous-rotation steer servos are also driven directly by
 * face buttons, configured with these names:
 *   "BR_Steer", "FR_Steer"
 * Cross (gamepad1.a)  -> BR_Steer spins at STEER_TEST_POWER while held
 * Square (gamepad1.x) -> FR_Steer spins at STEER_TEST_POWER while held
 * Releasing the button stops the corresponding servo.
 */
@TeleOp(name = "Motor Test Joystick TeleOp", group = "Test")
public class ahh extends LinearOpMode {

    // Hard cap on motor power - 0.5 means "full stick" only ever
    // produces half of the motor's actual max speed.
    private static final double MAX_SPEED = 0.5;

    // Software current limit. If a motor's draw exceeds this, its
    // power is cut to 0 until the draw falls back under the limit.
    // Tune this to whatever is safe for your specific motor.
    private static final double CURRENT_LIMIT_AMPS = 5.0;

    // Power used to spin the steer servos while their button is held.
    private static final double STEER_TEST_POWER = 0.5;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private AnalogInput frontRightEncoder;
    private AnalogInput backRightEncoder;

    private CRServo frontRightSteer;
    private CRServo backRightSteer;

    @Override
    public void runOpMode() {

        // Map hardware. Update the config names here if yours differ.
        frontRight = hardwareMap.get(DcMotorEx.class, "FR_Drive");
        backRight  = hardwareMap.get(DcMotorEx.class, "BR_Drive");
        frontLeft  = hardwareMap.get(DcMotorEx.class, "FL_Drive");
        backLeft   = hardwareMap.get(DcMotorEx.class, "BL_Drive");

        frontRightEncoder = hardwareMap.get(AnalogInput.class, "FR_Position");
        backRightEncoder  = hardwareMap.get(AnalogInput.class, "BR_Position");

        frontRightSteer = hardwareMap.get(CRServo.class, "FR_Steer");
        backRightSteer  = hardwareMap.get(CRServo.class, "BR_Steer");
        frontRightSteer.setPower(0);
        backRightSteer.setPower(0);

        for (DcMotorEx motor : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(CURRENT_LIMIT_AMPS, CurrentUnit.AMPS);
        }

        telemetry.addLine("Motor Test Ready");
        telemetry.addLine("Push left stick any direction - all motors match its magnitude");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Joystick inputs. Y is inverted because gamepad forward = negative.
            double y = -gamepad1.left_stick_y;
            double x =  gamepad1.left_stick_x;

            // Whichever axis is pushed further determines the power.
            // No direction/rotation logic - every motor just gets this
            // one value, so full right and full forward look identical
            // to the motors. Scaled down by MAX_SPEED so full stick
            // deflection never exceeds the configured speed cap.
            double power = (Math.abs(x) > Math.abs(y) ? x : y) * MAX_SPEED;

            // Apply the same requested power to every motor, but clamp
            // any individual motor to 0 if it's currently pulling more
            // than CURRENT_LIMIT_AMPS. This protects each motor
            // independently - one overloaded motor won't affect the others.
            double frontLeftPower  = frontLeft.isOverCurrent()  ? 0 : power;
            double frontRightPower = frontRight.isOverCurrent() ? 0 : power;
            double backLeftPower   = backLeft.isOverCurrent()   ? 0 : power;
            double backRightPower  = backRight.isOverCurrent()  ? 0 : power;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Analog encoders report position as a voltage (0V to the
            // controller's max, typically 3.3V for FTC's analog ports).
            // Dividing by max voltage gives a 0.0-1.0 fraction of a full
            // rotation, which is easier to read at a glance.
            double frMaxVoltage = frontRightEncoder.getMaxVoltage();
            double brMaxVoltage = backRightEncoder.getMaxVoltage();
            double frVoltage = frontRightEncoder.getVoltage();
            double brVoltage = backRightEncoder.getVoltage();
            double frPosition = frVoltage / frMaxVoltage;
            double brPosition = brVoltage / brMaxVoltage;

            // Cross (X) -> BR_Steer, Square ([]) -> FR_Steer. Each servo
            // spins at STEER_TEST_POWER only while its button is held,
            // and stops the instant the button is released.
            backRightSteer.setPower(gamepad1.a ? STEER_TEST_POWER : 0);
            frontRightSteer.setPower(gamepad1.x ? STEER_TEST_POWER : 0);

            telemetry.addData("Stick X", "%.2f", x);
            telemetry.addData("Stick Y", "%.2f", y);
            telemetry.addData("Motor Power (requested)", "%.2f", power);
            telemetry.addData("FL Current", "%.2fA  overCurrent=%b", frontLeft.getCurrent(CurrentUnit.AMPS), frontLeft.isOverCurrent());
            telemetry.addData("FR Current", "%.2fA  overCurrent=%b", frontRight.getCurrent(CurrentUnit.AMPS), frontRight.isOverCurrent());
            telemetry.addData("BL Current", "%.2fA  overCurrent=%b", backLeft.getCurrent(CurrentUnit.AMPS), backLeft.isOverCurrent());
            telemetry.addData("BR Current", "%.2fA  overCurrent=%b", backRight.getCurrent(CurrentUnit.AMPS), backRight.isOverCurrent());
            telemetry.addData("FR Position", "%.3fV  (%.1f%% of range)", frVoltage, frPosition * 100);
            telemetry.addData("BR Position", "%.3fV  (%.1f%% of range)", brVoltage, brPosition * 100);
            telemetry.addData("BR Steer (Cross)", "power=%.2f", backRightSteer.getPower());
            telemetry.addData("FR Steer (Square)", "power=%.2f", frontRightSteer.getPower());
            telemetry.update();
        }

        // Safety stop on exit.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        frontRightSteer.setPower(0);
        backRightSteer.setPower(0);
    }
}