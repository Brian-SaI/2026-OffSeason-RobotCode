package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Field Centric Swerve Test")
public class FieldCentricTeleOp extends OpMode {

    private Follower follower;

    // EMA (exponential moving average) smoothing for stick inputs.
    // alpha closer to 1.0 = less smoothing (more responsive, more jitter passes through)
    // alpha closer to 0.0 = more smoothing (less jitter, but more input lag)
    // Start at 0.3 and adjust to taste -- this is the main knob to tweak.
    private static final double SMOOTHING_ALPHA = 0.3;
    private double smoothedForward = 0;
    private double smoothedStrafe = 0;
    private double smoothedTurn = 0;

    private double smooth(double previous, double target) {
        return previous + SMOOTHING_ALPHA * (target - previous);
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Starting pose matters for field-centric -- heading here is what "forward" means
        // to the field-centric math. (0,0,0) is fine just for feel-testing drive.
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();
    }

    @Override
    public void start() {
        // true = use brake mode on the drive motors while driver-controlled
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        smoothedForward = smooth(smoothedForward, -gamepad1.left_stick_y);
        smoothedStrafe = smooth(smoothedStrafe, -gamepad1.left_stick_x);
        smoothedTurn = smooth(smoothedTurn, -gamepad1.right_stick_x);

        follower.setTeleOpDrive(
                smoothedForward,
                smoothedStrafe,
                smoothedTurn,
                false                    // false = FIELD CENTRIC
        );

        follower.update();

        telemetry.addData("pose", follower.getPose());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}