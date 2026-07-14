package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp for a single coaxial swerve module with NO position feedback of any kind
 * on the steering axis (raw CR servo only).
 *
 * Since the servo can't report where it's pointing, this code tracks an ESTIMATED
 * angle by integrating commanded servo power over time, using a calibrated
 * deg/sec-at-full-power constant. This is open-loop dead-reckoning, not true
 * closed-loop control:
 *
 *   - It WILL drift over the course of a match (collisions, slippage, friction,
 *     battery sag all throw off the estimate).
 *   - It requires the driver to physically point the module straight forward
 *     and press a button to "home" (zero) the estimate before each match.
 *   - You must calibrate MAX_SERVO_DEG_PER_SEC for your specific servo/module
 *     (see calibration note below).
 *
 * If precision steering matters to you, strongly consider adding a cheap analog
 * absolute encoder to the steer axis later -- that removes the drift problem
 * entirely. This version is a starting point for testing without one.
 *
 * Hardware assumed:
 *   - "driveMotor" : DcMotor driving the wheel
 *   - "steerServo" : CRServo rotating the module
 */
@TeleOp(name = "Single Swerve Module (No Feedback)", group = "Swerve")
public class a extends LinearOpMode {

    private DcMotor driveMotor;
    private CRServo steerServo;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ---- CALIBRATION ----
    // Command the servo at full power (1.0) for exactly 1 second and physically
    // measure how many degrees the module rotates. Put that number here.
    // This will vary with battery voltage and load, so treat it as an approximation`.
    private static final double MAX_SERVO_DEG_PER_SEC = 455.6; // <-- MEASURE AND SET THIS

    // Tuning constants
    private static final double STEER_KP = 0.010;
    private static final double STEER_MIN_POWER = 0.06;
    private static final double ANGLE_TOLERANCE = 1.0; // a bit looser since estimate is imprecise

    // Dead-reckoned steering angle estimate, degrees, normalized (-180, 180]
    private double estimatedAngle = 0.0;

    @Override
    public void runOpMode() {

        driveMotor = hardwareMap.get(DcMotor.class, "myMotor");
        steerServo = hardwareMap.get(CRServo.class, "myServo");
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Point the module STRAIGHT FORWARD by hand,");
        telemetry.addLine("then press A to zero the angle estimate.");
        telemetry.update();

        // Homing loop before match start
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                estimatedAngle = 0.0;
                telemetry.addLine("Homed! Angle estimate zeroed.");
                telemetry.update();
            }
        }

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {

            double dt = loopTimer.seconds();
            loopTimer.reset();

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double magnitude = Math.hypot(x, y);

            // Re-home mid-match if needed (e.g. if driver notices drift and can
            // manually confirm the module is pointing forward)
            if (gamepad1.a) {
                estimatedAngle = 0.0;
            }

            double servoPower = 0.0;

            if (magnitude > 0.05) {
                double targetAngle = Math.toDegrees(Math.atan2(x, y));
                targetAngle = normalizeAngle(targetAngle);

                double error = normalizeAngle(targetAngle - estimatedAngle);
                double driveDirection = 1.0;

                if (Math.abs(error) > 90.0) {
                    targetAngle = normalizeAngle(targetAngle + 180.0);
                    error = normalizeAngle(targetAngle - estimatedAngle);
                    driveDirection = -1.0;
                }

                servoPower = computeSteerPower(error);
                driveMotor.setPower(Range.clip(magnitude, 0, 0.5) * driveDirection);
            } else {
                driveMotor.setPower(0);
                servoPower = 0.0;
            }

            steerServo.setPower(servoPower);

            // Update the angle estimate based on what we just commanded
            estimatedAngle = normalizeAngle(
                    estimatedAngle + servoPower * MAX_SERVO_DEG_PER_SEC * dt);

            telemetry.addData("Estimated Angle (deg)", "%.1f", estimatedAngle);
            telemetry.addData("Servo Power", "%.2f", servoPower);
            telemetry.addLine("Press A anytime to re-home to forward");
            telemetry.update();
        }
    }

    /** Normalizes any angle in degrees to the range (-180, 180]. */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    /** Proportional controller producing a servo power command from angle error. */
    private double computeSteerPower(double error) {
        if (Math.abs(error) < ANGLE_TOLERANCE) {
            return 0.0;
        }

        double power = error * STEER_KP;
        power = Range.clip(power, -1.0, 1.0);

        if (Math.abs(power) < STEER_MIN_POWER) {
            power = Math.signum(power) * STEER_MIN_POWER;
        }

        return power;
    }
}

//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
///**
// * TeleOp for a single coaxial swerve module with NO position feedback of any kind
// * on the steering axis (raw CR servo only).
// *
// * Since the servo can't report where it's pointing, this code tracks an ESTIMATED
// * angle by integrating commanded servo power over time, using a calibrated
// * deg/sec-at-full-power constant. This is open-loop dead-reckoning, not true
// * closed-loop control:
// *
// *   - It WILL drift over the course of a match (collisions, slippage, friction,
// *     battery sag all throw off the estimate).
// *   - It requires the driver to physically point the module straight forward
// *     and press a button to "home" (zero) the estimate before each match.
// *   - You must calibrate MAX_SERVO_DEG_PER_SEC for your specific servo/module
// *     (see calibration note below).
// *
// * If precision steering matters to you, strongly consider adding a cheap analog
// * absolute encoder to the steer axis later -- that removes the drift problem
// * entirely. This version is a starting point for testing without one.
// *
// * Hardware assumed:
// *   - "driveMotor" : DcMotor driving the wheel
// *   - "steerServo" : CRServo rotating the module
// */
//@TeleOp(name = "Single Swerve Module (No Feedback)", group = "Swerve")
//public class a extends LinearOpMode {
//
//    private DcMotor driveMotor;
//    private CRServo steerServo;
//    private final ElapsedTime loopTimer = new ElapsedTime();
//
//    // ---- CALIBRATION ----
//    // Command the servo at full power (1.0) for exactly 1 second and physically
//    // measure how many degrees the module rotates. Put that number here.
//    // This will vary with battery voltage and load, so treat it as an approximation`.
//    private static final double MAX_SERVO_DEG_PER_SEC = 180 ; //428.6; // <-- MEASURE AND SET THIS
//
//    // Tuning constants
//    private static final double STEER_KP = 0.010;
//    private static final double STEER_MIN_POWER = 0.06;
//    private static final double ANGLE_TOLERANCE = 3.0; // a bit looser since estimate is imprecise
//
//    // Dead-reckoned steering angle estimate, degrees, normalized (-180, 180]
//    private double estimatedAngle = 0.0;
//
//    @Override
//    public void runOpMode() {
//
//        driveMotor = hardwareMap.get(DcMotor.class, "myMotor");
//        steerServo = hardwareMap.get(CRServo.class, "myServo");
//        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        telemetry.addLine("Point the module STRAIGHT FORWARD by hand,");
//        telemetry.addLine("then press A to zero the angle estimate.");
//        telemetry.update();
//
//        // Homing loop before match start
//        while (!isStarted() && !isStopRequested()) {
//            if (gamepad1.a) {
//                estimatedAngle = 0.0;
//                telemetry.addLine("Homed! Angle estimate zeroed.");
//                telemetry.update();
//            }
//        }
//
//        waitForStart();
//        loopTimer.reset();
//
//        while (opModeIsActive()) {
//
//            double dt = loopTimer.seconds();
//            loopTimer.reset();
//
//            double x = gamepad1.left_stick_x;
//            double y = -gamepad1.left_stick_y;
//            double magnitude = Math.hypot(x, y);
//
//            // Re-home mid-match if needed (e.g. if driver notices drift and can
//            // manually confirm the module is pointing forward)
//            if (gamepad1.a) {
//                estimatedAngle = 0.0;
//            }
//
//            double servoPower = 0.0;
//
//            if (magnitude > 0.05) {
//                double targetAngle = Math.toDegrees(Math.atan2(x, y));
//                targetAngle = normalizeAngle(targetAngle);
//
//                double error = normalizeAngle(targetAngle - estimatedAngle);
//                double driveDirection = 1.0;
//
//                if (Math.abs(error) > 90.0) {
//                    targetAngle = normalizeAngle(targetAngle + 180.0);
//                    error = normalizeAngle(targetAngle - estimatedAngle);
//                    driveDirection = -1.0;
//                }
//
//                servoPower = computeSteerPower(error);
//                driveMotor.setPower(Range.clip(magnitude, 0, 0.5) * driveDirection);
//            } else {
//                driveMotor.setPower(0);
//                servoPower = 0.0;
//            }
//
//            steerServo.setPower(servoPower);
//
//            // Update the angle estimate based on what we just commanded
//            estimatedAngle = normalizeAngle(
//                    estimatedAngle + servoPower * MAX_SERVO_DEG_PER_SEC * dt);
//
//            telemetry.addData("Estimated Angle (deg)", "%.1f", estimatedAngle);
//            telemetry.addData("Servo Power", "%.2f", servoPower);
//            telemetry.addLine("Press A anytime to re-home to forward");
//            telemetry.update();
//        }
//    }
//
//    /** Normalizes any angle in degrees to the range (-180, 180]. */
//    private double normalizeAngle(double angle) {
//        while (angle > 180) angle -= 360;
//        while (angle <= -180) angle += 360;
//        return angle;
//    }
//
//    /** Proportional controller producing a servo power command from angle error. */
//    private double computeSteerPower(double error) {
//        if (Math.abs(error) < ANGLE_TOLERANCE) {
//            return 0.0;
//        }
//
//        double power = error * STEER_KP;
//        power = Range.clip(power, -1.0, 1.0);
//
//        if (Math.abs(power) < STEER_MIN_POWER) {
//            power = Math.signum(power) * STEER_MIN_POWER;
//        }
//
//        return power;
//    }
//}
////
////package org.firstinspires.ftc.teamcode;
////
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.CRServo;
////import com.qualcomm.robotcore.util.Range;
////
/////**
//// * TeleOp for a single coaxial swerve module, steered by a CR servo, with a REV
//// * Through Bore Encoder plugged into a motor port's encoder pins ("myEncoder")
//// * to provide real steering position feedback.
//// *
// * The Through Bore Encoder here is used in its RELATIVE (quadrature) mode -- it
// * reports precise tick counts, but it does NOT know absolute position on power-up.
// * That means:
// *
// *   - No drift while the code is running (unlike the pure time-based estimate
// *     from the no-feedback version) -- this is real closed-loop control.
// *   - You still need to home it once per match: point the module straight
// *     forward by hand, then press A to zero the reading.
// *   - If you lose power to the hub mid-match or the encoder cable disconnects,
// *     you'll need to re-home.
// *
// * Hardware assumed:
// *   - "driveMotor" : DcMotor driving the wheel
// *   - "steerServo" : CRServo rotating the module
// *   - "myEncoder"  : DcMotor slot used ONLY for its encoder (REV Through Bore
// *                    plugged into that port's encoder pins). Never powered.
// */
//@TeleOp(name = "Single Swerve Module (Through Bore Encoder)", group = "Swerve")
//public class a extends LinearOpMode {
//
//    private DcMotor driveMotor;
//    private CRServo steerServo;
//    private DcMotor steerEncoder; // read-only, never powered
//
//    // ---- CALIBRATION ----
//    // REV Through Bore Encoder: 8192 counts per revolution of the encoder shaft.
//    private static final double ENCODER_COUNTS_PER_REV = 8192.0;
//
//    // If the encoder shaft is geared/belted down (or up) from the actual module
//    // rotation axis, set this to (module revolutions per 1 encoder shaft revolution).
//    // Direct 1:1 coupling (encoder shaft = module axis) -> leave at 1.0.
//    private static final double ENCODER_TO_MODULE_RATIO = 1.0; // <-- SET FOR YOUR MECHANISM
//
//    private static final double COUNTS_PER_MODULE_REV =
//            ENCODER_COUNTS_PER_REV / ENCODER_TO_MODULE_RATIO;
//
//    // Tuning constants
//    private static final double STEER_KP = 0.010;
//    private static final double STEER_MIN_POWER = 0.06;
//    private static final double ANGLE_TOLERANCE = 2.0; // real feedback -> can be tighter than before
//
//    // Homing offset, in encoder ticks, subtracted from raw position to get "forward = 0"
//    private int homeOffsetTicks = 0;
//
//    @Override
//    public void runOpMode() {
//
//        driveMotor = hardwareMap.get(DcMotor.class, "myMotor");
//        steerServo = hardwareMap.get(CRServo.class, "myServo");
//        steerEncoder = hardwareMap.get(DcMotor.class, "myEncoder");
//
//        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // If the module reads backwards after testing, uncomment the next line:
//        // steerEncoder.setDirection(DcMotor.Direction.REVERSE);
//
//        telemetry.addLine("Point the module STRAIGHT FORWARD by hand,");
//        telemetry.addLine("then press A to zero the angle.");
//        telemetry.update();
//
//        // Homing loop before match start
//        while (!isStarted() && !isStopRequested()) {
//            if (gamepad1.a) {
//                homeOffsetTicks = steerEncoder.getCurrentPosition();
//                telemetry.addLine("Homed!");
//                telemetry.update();
//            }
//        }
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            double x = gamepad1.left_stick_x;
//            double y = -gamepad1.left_stick_y;
//            double magnitude = Math.hypot(x, y);
//
//            // Re-home mid-match if needed
//            if (gamepad1.a) {
//                homeOffsetTicks = steerEncoder.getCurrentPosition();
//            }
//
//            double currentAngle = getCurrentAngle();
//
//            if (magnitude > 0.05) {
//                double targetAngle = Math.toDegrees(Math.atan2(x, y));
//                targetAngle = normalizeAngle(targetAngle);
//
//                double error = normalizeAngle(targetAngle - currentAngle);
//                double driveDirection = 1.0;
//
//                if (Math.abs(error) > 90.0) {
//                    targetAngle = normalizeAngle(targetAngle + 180.0);
//                    error = normalizeAngle(targetAngle - currentAngle);
//                    driveDirection = -1.0;
//                }
//
//                steerToAngle(error);
//                driveMotor.setPower(Range.clip(magnitude, 0, 0.5) * driveDirection);
//
//            } else {
//                driveMotor.setPower(0);
//                steerServo.setPower(0);
//            }
//
//            telemetry.addData("Current Angle (deg)", "%.1f", currentAngle);
//            telemetry.addData("Raw Ticks", steerEncoder.getCurrentPosition());
//            telemetry.addLine("Press A anytime to re-home to forward");
//            telemetry.update();
//        }
//    }
//
//    /** Converts homed encoder ticks into a steering angle in degrees, normalized (-180, 180]. */
//    private double getCurrentAngle() {
//        int ticks = steerEncoder.getCurrentPosition() - homeOffsetTicks;
//        double angle = (ticks / COUNTS_PER_MODULE_REV) * 360.0;
//        return normalizeAngle(angle);
//    }
//
//    /** Normalizes any angle in degrees to the range (-180, 180]. */
//    private double normalizeAngle(double angle) {
//        while (angle > 180) angle -= 360;
//        while (angle <= -180) angle += 360;
//        return angle;
//    }
//
//    /** Proportional controller driving the CR servo toward zero error. */
//    private void steerToAngle(double error) {
//        if (Math.abs(error) < ANGLE_TOLERANCE) {
//            steerServo.setPower(0);
//            return;
//        }
//
//        double power = error * STEER_KP;
//        power = Range.clip(power, -1.0, 1.0);
//
//        if (Math.abs(power) < STEER_MIN_POWER) {
//            power = Math.signum(power) * STEER_MIN_POWER;
//        }
//
//        steerServo.setPower(power);
//    }
//}