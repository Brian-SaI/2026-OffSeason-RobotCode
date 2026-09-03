package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.VectorCalculator;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // ===================== FollowerConstants =====================
    // These get tuned later, in the Localization / Velocity / Heading tuning pages.
    // Left at conservative placeholders for now -- do NOT trust these until tuned.
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-30) // TODO: tune via ForwardZeroPowerAccelerationTuner
            .lateralZeroPowerAcceleration(-30) // TODO: swerve -> same as forward, no separate lateral tuner needed
            .useSecondaryDrivePIDF(true).useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)

            .translationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0)) // TODO: tune (Heading/Translational tuning)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0)) // TODO: tune

            .headingPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0)) // TODO: tune
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0)) // TODO: tune

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.00003, 0.6, 0.13)) // restored -- previously tuned
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.004, 0, 0.000002, 0.6, 0.13)) // restored -- previously tuned

            // Docs recommend NOT using F values on heading/translational PIDs for swerve --
            // they tend to cause oscillation since pods can't change direction instantaneously.

            // Predictive braking is NOT compatible with swerve currently -- leave disabled.
            // .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.05, 0, 0.002))

            .centripetalScaling(0.0005) // docs note a very low value (~0.0005) helps minimize spline oscillation on swerve
            .mass(13.732); // TODO: update to your robot's actual mass (kg)

    // ===================== Localizer (Pinpoint) =====================
    // Keep your measured odometry pod offsets -- these come from physical measurement, not tuning.
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.9350393701) // TODO: update to your measured value (in.)
            .strafePodX(-5.9133858268) // TODO: update to your measured value (in.)
            .distanceUnit(DistanceUnit.INCH).hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // ===================== Swerve Constants =====================
    public static SwerveConstants swerveConstants = new SwerveConstants()
            .maxPower(1); // determines the max power of the drivetrain
            // Disables x-locking, useful while tuning pod offsets. Re-enable (comment this out)
            // once Angle Offset / Motor Direction / Encoder Direction tuning is done.
            //.zeroPowerBehavior(SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
    // .useBrakeModeInTeleOp(true) // optional, add back once you decide you want it

    // ===================== Pod PIDF placeholders =====================
    // Docs: "put in the placeholder pod PIDF coefficients (kP = 0.3, kI = 0, kD = 0.005, kF = 0)"
    // before running Swerve Offsets Test. Tune these for real in the Pod PIDF Tuning step.
    private static double kP = 0.3;
    private static double kD = 0.005;
    private static double kFFront = 0;
    private static double kFBack = 0;

    // Measured robot dimensions -- not tuned, just measure your drivetrain.
    private static double dtLength = 159.77; // measured (CAD), fore/aft offset
    private static double dtWidth = 137.06;  // measured (CAD), left/right offset

    private static CoaxialPod leftFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "FL_Drive", "FL_Steer", "FL_Position",
                new PIDFCoefficients(kP, 0, kD, kFFront),
                DcMotorSimple.Direction.FORWARD, // TODO: confirm via Motor Directions step
                DcMotorSimple.Direction.FORWARD, // TODO: confirm via Motor Directions step (servo)
                0.24999888910144346, // angle offset in radians -- tuned
                new Pose(dtLength, dtWidth),
                0.019, 3.211, // analog min/max -- already tuned
                false); // TODO: encoder inverted, from Encoder Directions step
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "FR_Drive", "FR_Steer", "FR_Position",
                new PIDFCoefficients(kP, 0, kD, kFFront),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                2.987794697052968,
                new Pose(dtLength, -dtWidth),
                0.006, 3.213, // analog min/max -- already tuned
                false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod leftBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "BL_Drive", "BL_Steer", "BL_Position",
                new PIDFCoefficients(kP, 0, kD, kFBack),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                2.8300865944130758,
                new Pose(-dtLength, dtWidth),
                0.004, 3.201, // analog min/max -- already tuned
                false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "BR_Drive", "BR_Steer", "BR_Position",
                new PIDFCoefficients(kP, 0, kD, kFBack),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                0.10178368721910855,
                new Pose(-dtLength, -dtWidth),
                0.006, 3.216, // analog min/max -- already tuned
                false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    // ===================== Path Constraints =====================
    // Pedro default -- fine to leave as-is until you're deep into path-following tuning.
    public static PathConstraints pathConstraints =
            new PathConstraints(0.995, 0.1, 0.1, 0.007, 100, 1, 10, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap).pathConstraints(pathConstraints)
                .swerveDrivetrain(swerveConstants, leftFront(hardwareMap), rightFront(hardwareMap),
                        leftBack(hardwareMap), rightBack(hardwareMap))
                .pinpointLocalizer(localizerConstants).build();
    }
}