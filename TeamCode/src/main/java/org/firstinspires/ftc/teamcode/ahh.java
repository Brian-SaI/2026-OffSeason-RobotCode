package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "CR Servo Triangle Test", group = "Test")
public class ahh extends LinearOpMode {

    private CRServo myServo;
    private DcMotorEx myMotor;

    // Set this to your motor's actual max ticks-per-second at full speed.
    // Example: a GoBILDA 5203 (312 RPM, 537.7 ticks/rev) maxes out around 2800 ticks/sec.
    // Check your motor's spec sheet to get an accurate number.
    private static final double MAX_VELOCITY_TICKS_PER_SEC = 2800.0;

    @Override
    public void runOpMode() {
        // Map the servo to the name configured in your robot configuration
        myServo = hardwareMap.get(CRServo.class, "myServo");
        myMotor = hardwareMap.get(DcMotorEx.class, "myMotor");

        // RUN_USING_ENCODER is required for setVelocity() to work (closed-loop control)
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Servo control ---
            if (gamepad1.triangle) {
                myServo.setPower(1.0);  // full speed forward
            } else if (gamepad1.cross) {
                myServo.setPower(-1.0); // full speed reverse
            } else {
                myServo.setPower(0.0);  // stop
            }

            // --- Motor control (closed-loop velocity) ---
            if (gamepad1.circle) {
                myMotor.setVelocity(MAX_VELOCITY_TICKS_PER_SEC);  // full speed forward
            } else if (gamepad1.square) {
                myMotor.setVelocity(-MAX_VELOCITY_TICKS_PER_SEC); // full speed reverse
            } else {
                myMotor.setVelocity(0.0);  // stop
            }

            telemetry.addData("Triangle pressed", gamepad1.triangle);
            telemetry.addData("Servo power", myServo.getPower());
            telemetry.addData("Motor target velocity", MAX_VELOCITY_TICKS_PER_SEC);
            telemetry.addData("Motor actual velocity", myMotor.getVelocity());
            telemetry.update();
        }
    }
}