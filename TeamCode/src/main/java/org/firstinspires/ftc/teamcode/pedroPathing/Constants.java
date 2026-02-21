package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.0)
            .forwardZeroPowerAcceleration(-30.82580388673541)
            .lateralZeroPowerAcceleration(-61.14929460923046)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.167,0,0.01,0.02167))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.1,0.02))
            .centripetalScaling(0.005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.3)
            .rightFrontMotorName("FrontRight")
            .rightRearMotorName("RearRight")
            .leftRearMotorName("RearLeft")
            .leftFrontMotorName("FrontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(62.69464491378145)
            .yVelocity(49.594345284340505);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.001958675)
            .strafeTicksToInches(0.0019602)
            .turnTicksToInches(0.001983)
            .leftPodY(3.5)
            .rightPodY(-2.5)//needs to be measured
            .strafePodX(0.0625)
            .leftEncoder_HardwareMapName("FrontRight")
            .rightEncoder_HardwareMapName("FrontLeft")
            .strafeEncoder_HardwareMapName("RearRight")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}