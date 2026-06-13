package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-45.400846586442514)
            .lateralZeroPowerAcceleration(-70.22445936114721)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0, 0.01, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0.0,0.0001,0.6,0.01))

            .mass(11);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.95, 0.6);
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .strafePodX(-7.805)
            .forwardPodY(3.695)
            .forwardEncoder_HardwareMapName("BackR")
            .strafeEncoder_HardwareMapName("FrontR")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            )
            .forwardTicksToInches(0.001998805694127871)
            .strafeTicksToInches(0.001989460559462039);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)

                .build();
    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.3)
            .rightFrontMotorName("FrontR")
            .rightRearMotorName("BackR")
            .leftRearMotorName("BackL")
            .leftFrontMotorName("FrontL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(66.47046712596801)
            .yVelocity(53.260846037344244)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
}