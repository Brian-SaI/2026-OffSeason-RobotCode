package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * MotorTestTeleOp
 *
 * A simple diagnostic TeleOp used to verify that four motors are wired and
 * spinning correctly. Each motor is triggered by one of the PlayStation
 * controller's face buttons. Hold a button down to run the matching motor
 * forward at a safe test speed; release the button to stop it.
 *
 * PlayStation button -> gamepad field mapping (FTC SDK):
 *   Cross    (X) -> gamepad1.a
 *   Circle   (O) -> gamepad1.b
 *   Square   ([]) -> gamepad1.x
 *   Triangle (^) -> gamepad1.y
 *
 * Configure four motors in your robot configuration with these names
 * (or change the strings below to match your own configuration):
 *   "motor1", "motor2", "motor3", "motor4"
 */
@TeleOp(name = "Motor Hardware Test", group = "Test")
public class ahh extends LinearOpMode {

    // Speed used for testing. Kept modest so motors don't take off unexpectedly.
    private static final double TEST_POWER = 1;

    private DcMotor motor1; // Cross    (X)
    private DcMotor motor2; // Circle   (O)
    private DcMotor motor3; // Square   ([])
    private DcMotor motor4; // Triangle (^)

    @Override
    public void runOpMode() {

        // Map hardware. Update the config names here if yours differ.
        motor1 = hardwareMap.get(DcMotor.class, "FR_Drive");
        motor2 = hardwareMap.get(DcMotor.class, "BR_Drive");
        motor3 = hardwareMap.get(DcMotor.class, "FL_Drive");
        motor4 = hardwareMap.get(DcMotor.class, "BL_Drive");

        // Make sure motors are stopped and braking when not powered.
        for (DcMotor motor : new DcMotor[]{motor1, motor2, motor3, motor4}) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Motor Test Ready");
        telemetry.addLine("Cross=motor1  Circle=motor2  Square=motor3  Triangle=motor4");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Cross (X) button -> motor1
            motor1.setPower(gamepad1.a ? TEST_POWER : 0);

            // Circle (O) button -> motor2
            motor2.setPower(gamepad1.b ? TEST_POWER : 0);

            // Square ([]) button -> motor3
            motor3.setPower(gamepad1.x ? TEST_POWER : 0);

            // Triangle (^) button -> motor4
            motor4.setPower(gamepad1.y ? TEST_POWER : 0);

            telemetry.addData("Motor 1 (Cross)",    "power=%.2f", motor1.getPower());
            telemetry.addData("Motor 2 (Circle)",   "power=%.2f", motor2.getPower());
            telemetry.addData("Motor 3 (Square)",   "power=%.2f", motor3.getPower());
            telemetry.addData("Motor 4 (Triangle)", "power=%.2f", motor4.getPower());
            telemetry.update();
        }

        // Safety stop on exit.
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
}