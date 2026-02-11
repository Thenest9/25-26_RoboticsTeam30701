package org.firstinspires.ftc.teamcode.pedroPathing;

//------------------------GETTING IMPORTS------------------------//
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class BaseStartRed9Ball
{
    public static class Paths
    {
        public PathChain firstShoot;
        public PathChain moveFirstRow;
        public PathChain collectFirstRow;
        public PathChain secondShot;
        public PathChain moveSecondRow;
        public PathChain collectSecondRow;
        public PathChain thirdShot;

        public Paths(Follower follower) {
            firstShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.500, 120.000),

                                    new Pose(96.000, 95.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))

                    .build();

            moveFirstRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 95.500),

                                    new Pose(96.000, 56.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            collectFirstRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 56.000),

                                    new Pose(125.000, 56.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            secondShot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.000, 56.000),
                                    new Pose(96.000, 56.012),
                                    new Pose(96.000, 95.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            moveSecondRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 95.500),

                                    new Pose(96.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            collectSecondRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 80.000),

                                    new Pose(125.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            thirdShot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 80.000),

                                    new Pose(96.000, 95.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();
        }
    }

}
