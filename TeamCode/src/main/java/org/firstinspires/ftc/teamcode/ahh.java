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
 * Controls:
 *   Gamepad 1:
 *     - Left stick (X/Y) -> Drives ALL 4 motors at once based on stick magnitude.
 *     - Cross (A)    -> Spins BR_Steer servo
 *     - Square (X)   -> Spins FR_Steer servo
 *     - Triangle (Y) -> Spins FL_Steer servo
 *     - Circle (B)   -> Spins BL_Steer servo
 *
 *   Gamepad 2:
 *     - Cross (A)    -> Spins BR_Drive motor
 *     - Square (X)   -> Spins FR_Drive motor
 *     - Triangle (Y) -> Spins FL_Drive motor
 *     - Circle (B)   -> Spins BL_Drive motor
 */
@TeleOp(name = "Motor Test Joystick TeleOp", group = "Test")
public class ahh extends LinearOpMode {

    // Hard cap on motor power
    private static final double MAX_SPEED = 0.85;

    // Power used to spin an individual motor via Gamepad 2 buttons
    private static final double INDIVIDUAL_MOTOR_POWER = 1.0;

    // Software current limit in Amperes
    private static final double CURRENT_LIMIT_AMPS = 5.0;

    // Power used to spin the steer servos while their button is held
    private static final double STEER_TEST_POWER = 0.5;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private AnalogInput frontRightEncoder;
    private AnalogInput backRightEncoder;
    private AnalogInput frontLeftEncoder;
    private AnalogInput backLeftEncoder;

    private CRServo frontRightSteer;
    private CRServo backRightSteer;
    private CRServo frontLeftSteer;
    private CRServo backLeftSteer;

    @Override
    public void runOpMode() {

        // Map drive motors
        frontRight = hardwareMap.get(DcMotorEx.class, "FR_Drive");
        backRight  = hardwareMap.get(DcMotorEx.class, "BR_Drive");
        frontLeft  = hardwareMap.get(DcMotorEx.class, "FL_Drive");
        backLeft   = hardwareMap.get(DcMotorEx.class, "BL_Drive");

        // Map analog position encoders
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "FR_Position");
        backRightEncoder  = hardwareMap.get(AnalogInput.class, "BR_Position");
        frontLeftEncoder  = hardwareMap.get(AnalogInput.class, "FL_Position");
        backLeftEncoder   = hardwareMap.get(AnalogInput.class, "BL_Position");

        // Map steer servos
        frontRightSteer = hardwareMap.get(CRServo.class, "FR_Steer");
        backRightSteer  = hardwareMap.get(CRServo.class, "BR_Steer");
        frontLeftSteer  = hardwareMap.get(CRServo.class, "FL_Steer");
        backLeftSteer   = hardwareMap.get(CRServo.class, "BL_Steer");

        frontRightSteer.setPower(0);
        backRightSteer.setPower(0);
        frontLeftSteer.setPower(0);
        backLeftSteer.setPower(0);

        for (DcMotorEx motor : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(CURRENT_LIMIT_AMPS, CurrentUnit.AMPS);
        }

        telemetry.addLine("Motor Test Ready");
        telemetry.addLine("G1 Left Stick: All motors together");
        telemetry.addLine("G1 A/X/Y/B: Individual servos");
        telemetry.addLine("G2 A/X/Y/B: Individual motors");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Gamepad 1 Joystick inputs
            double y = -gamepad1.left_stick_y;
            double x =  gamepad1.left_stick_x;

            // Power calculation for all motors driven by G1 stick
            double allMotorPower = (Math.abs(x) > Math.abs(y) ? x : y) * MAX_SPEED;

            // Determine individual motor target powers:
            // Check Gamepad 2 face buttons first; if none are pressed, fallback to Gamepad 1 stick control.
            double targetFL = gamepad2.y ? INDIVIDUAL_MOTOR_POWER : allMotorPower;
            double targetFR = gamepad2.x ? INDIVIDUAL_MOTOR_POWER : allMotorPower;
            double targetBL = gamepad2.b ? INDIVIDUAL_MOTOR_POWER : allMotorPower;
            double targetBR = gamepad2.a ? INDIVIDUAL_MOTOR_POWER : allMotorPower;

            // Overcurrent protection per motor
            double frontLeftPower  = frontLeft.isOverCurrent()  ? 0 : targetFL;
            double frontRightPower = frontRight.isOverCurrent() ? 0 : targetFR;
            double backLeftPower   = backLeft.isOverCurrent()   ? 0 : targetBL;
            double backRightPower  = backRight.isOverCurrent()  ? 0 : targetBR;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Read encoder voltages and calculate positions (0.0 to 1.0 fraction)
            double frVoltage = frontRightEncoder.getVoltage();
            double brVoltage = backRightEncoder.getVoltage();
            double flVoltage = frontLeftEncoder.getVoltage();
            double blVoltage = backLeftEncoder.getVoltage();

            double frPosition = frVoltage / frontRightEncoder.getMaxVoltage();
            double brPosition = brVoltage / backRightEncoder.getMaxVoltage();
            double flPosition = flVoltage / frontLeftEncoder.getMaxVoltage();
            double blPosition = blVoltage / backLeftEncoder.getMaxVoltage();

            // Gamepad 1 Button mapping for continuous rotation servos:
            // Cross (A)    -> BR_Steer
            // Square (X)   -> FR_Steer
            // Triangle (Y) -> FL_Steer
            // Circle (B)   -> BL_Steer
            backRightSteer.setPower(gamepad1.a ? STEER_TEST_POWER : 0);
            frontRightSteer.setPower(gamepad1.x ? STEER_TEST_POWER : 0);
            frontLeftSteer.setPower(gamepad1.y ? STEER_TEST_POWER : 0);
            backLeftSteer.setPower(gamepad1.b ? STEER_TEST_POWER : 0);

            // Telemetry
            telemetry.addData("Stick X (G1)", "%.2f", x);
            telemetry.addData("Stick Y (G1)", "%.2f", y);
            telemetry.addData("All Motor Power", "%.2f", allMotorPower);

            telemetry.addData("FL Motor (G2 Y)", "Pwr=%.2f  Curr=%.2fA  overCurrent=%b", frontLeft.getPower(), frontLeft.getCurrent(CurrentUnit.AMPS), frontLeft.isOverCurrent());
            telemetry.addData("FR Motor (G2 X)", "Pwr=%.2f  Curr=%.2fA  overCurrent=%b", frontRight.getPower(), frontRight.getCurrent(CurrentUnit.AMPS), frontRight.isOverCurrent());
            telemetry.addData("BL Motor (G2 B)", "Pwr=%.2f  Curr=%.2fA  overCurrent=%b", backLeft.getPower(), backLeft.getCurrent(CurrentUnit.AMPS), backLeft.isOverCurrent());
            telemetry.addData("BR Motor (G2 A)", "Pwr=%.2f  Curr=%.2fA  overCurrent=%b", backRight.getPower(), backRight.getCurrent(CurrentUnit.AMPS), backRight.isOverCurrent());

            telemetry.addData("FL Position", "%.3fV  (%.1f%%)", flVoltage, flPosition * 100);
            telemetry.addData("FR Position", "%.3fV  (%.1f%%)", frVoltage, frPosition * 100);
            telemetry.addData("BL Position", "%.3fV  (%.1f%%)", blVoltage, blPosition * 100);
            telemetry.addData("BR Position", "%.3fV  (%.1f%%)", brVoltage, brPosition * 100);

            telemetry.addData("FL Steer (G1 Y)", "power=%.2f", frontLeftSteer.getPower());
            telemetry.addData("FR Steer (G1 X)", "power=%.2f", frontRightSteer.getPower());
            telemetry.addData("BL Steer (G1 B)", "power=%.2f", backLeftSteer.getPower());
            telemetry.addData("BR Steer (G1 A)", "power=%.2f", backRightSteer.getPower());

            telemetry.update();
        }

        // Safety stop on exit
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        frontRightSteer.setPower(0);
        backRightSteer.setPower(0);
        frontLeftSteer.setPower(0);
        backLeftSteer.setPower(0);
    }
}