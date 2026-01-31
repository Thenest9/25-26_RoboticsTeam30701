package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawDebug;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Base Start Red", group = "Autonomous")
public class BaseStartRedPedro extends OpMode
{
    Limelight3A limelight;

    //Making the DcMotors for the tires, the ramp, and the intake
    DcMotor FrontLeft,FrontRight,RearLeft,RearRight, intake, ramp;
    //Making the DcMotorEx for the fly wheels
    DcMotorEx outputRight, outputLeft;

    //Making the colorSensor to be able to check the color of the balls
    ColorSensor colorSensor;

    //Making the CRServo for the carousel
    CRServo carousel;
    DigitalChannel touchSensorBot;
    DigitalChannel touchSensorTop;

    //Making the gate to have the gate open or closed depending on intake or output
    Servo gate;
    double shooterP = 48.72995;
    double shooterI = 0;
    double shooterD = 0;
    double shooterF = 13.13319;

    LibraryPedro lib;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState; // Current autonomous path state (state machine)
    private final Pose startPose= new Pose(122.69701280227595, 123.92603129445234, Math.toRadians(36));
    public Path shootingPose;
    public PathChain collect11, collect12, collect1SP, collect21, collect22, collect23, collect2SP;
    public PathChain path9, shootAngle;

    public String currMotif;

    public void buildPaths()
    {
        shootingPose = new Path(new BezierLine(new Pose(122.697, 123.926), new Pose(96.000, 95.518)));
        shootingPose.setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(90));

        shootAngle = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.000, 95.518),

                                new Pose(96.000, 95.618)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(40))

                .build();

        collect11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.000, 95.618),

                                new Pose(100.000, 83.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))

                .build();

        collect12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(100.000, 83.000),

                                new Pose(129.200, 83.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        collect1SP = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.200, 83.000),

                                new Pose(96.000, 95.518)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))

                .build();

        collect21 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.000, 95.518),

                                new Pose(100.000, 58.400)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                .build();

        collect22 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(100.000, 58.400),

                                new Pose(135.000, 58.400)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        collect2SP = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(135.000, 58.400),
                                new Pose(103.647, 54.896),
                                new Pose(96.000, 95.518)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))

                .build();

        path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.000, 95.518),

                                new Pose(125.000, 70.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(270))

                .build();
    }

    public void autonomousPathUpdate()
    {
        switch (pathState)
        {
            case 0:
                follower.followPath(shootingPose);
                currMotif = lib.getMotif();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy())
                {
                    follower.followPath(shootAngle,true);
                    lib.orderBalls(currMotif, "ppg");
                    lib.shootThree(1367);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && !lib.isShooting && pathTimer.getElapsedTimeSeconds()>6) {
                    lib.rampDown();
                    follower.followPath(collect11, true);
                    lib.IntakeStart();//ramp moves down, intake starts spinning, carousel spins continously
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2)
                {
                    follower.followPath(collect12, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() && !lib.isIntaking && pathTimer.getElapsedTimeSeconds()>1)
                {
                    actionTimer.resetTimer();
                    if(actionTimer.getElapsedTimeSeconds()>0.5)
                    {
                        lib.rampUp();
                    }
                    lib.runTogether(//TEST
                            ()-> lib.orderBalls(currMotif, "gpp"),
                            ()->follower.followPath(collect1SP, true)
                    );
                    lib.shootThree(1367);
                }
                break;
            case 5:
                if(!follower.isBusy()&& !lib.isShooting && pathTimer.getElapsedTimeSeconds()>3)
                {
                    lib.rampDown();
                    follower.followPath(collect21, true);
//                    lib.Intake();//starts intake, brings ramp down
                    setPathState(6);
                }
                break;
//            case 6:
//                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.5)
//                {
//                    follower.followPath(collect22,true);
//                    setPathState(7);
//                }
//                break;
////            case 7:
//                if(!follower.isBusy() && !lib.isIntaking && pathTimer.getElapsedTimeSeconds()>1)
//                {
//                    follower.followPath(collect2SP,true);
//                    lib.shootThree(1267);
//                    setPathState(9);
//                    }
//                    break;
//            case 9:
//                if (!follower.isBusy() && !lib.isShooting && pathTimer.getElapsedTimeSeconds()>3)
//                {
//                    follower.followPath(path9, true);
//                }
//                break;
            }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init()
    {
        // Initialize the right wheel of the fly wheel
        outputRight = hardwareMap.get(DcMotorEx.class, "RightOutput");
        outputRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outputRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the left wheel of the fly wheel
        outputLeft = hardwareMap.get(DcMotorEx.class, "LeftOutput");
        outputLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outputRight.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        outputLeft.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        // Initialize the intake motor
        intake = hardwareMap.get(DcMotor.class, "Intake");

        // Initialize the carousel servo
        carousel = hardwareMap.get(CRServo.class, "Carousel");

        // Initialize the ramp
        ramp = hardwareMap.get(DcMotor.class, "rampIntakeOuttake");

        // Initialize the gate
        gate = hardwareMap.get(Servo.class, "gate");

        // Initialize the color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Initialize the limelight and then setting the pipeline to 7 ( Pipeline for the Motif)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();


        touchSensorTop = hardwareMap.get(DigitalChannel.class, "touchSensorTop");
        touchSensorTop.setMode(DigitalChannel.Mode.INPUT);


        touchSensorBot = hardwareMap.get(DigitalChannel.class, "touchSensorDown");
        touchSensorBot.setMode(DigitalChannel.Mode.INPUT);

        // Initialize the library class through a lib object

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer =  new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower((hardwareMap));
        buildPaths();
        follower.setStartingPose(startPose);

        lib = new LibraryPedro(outputRight, outputLeft, carousel, telemetry, limelight, intake, ramp, gate, colorSensor, touchSensorTop, touchSensorBot);

    }


    @Override
    public void loop()//runs about 50 times a second
    {
        follower.update(); // Update Pedro Pathing
        lib.updateShoot();
        lib.finishIntake();
        if(lib.isIntaking)
        {
            if(lib.isBall()) {
                lib.carouselStart();
            }
            else
            {
                carousel.setPower(0);
            }
        }

        autonomousPathUpdate();
        drawDebug(follower);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Shoot timer: ", lib.getShootTimer());
        telemetry.addData("isShooting:", lib.isShooting);
        telemetry.addData("intake timer: ", lib.getIntakeTimer());
        telemetry.addData("Carousel timer: ", lib.getCarTimer());
        telemetry.addData("isIntaking:", lib.isIntaking);
        telemetry.addData("Motif", "hi");
        telemetry.addData("Bottom", touchSensorBot.getState());
        telemetry.addData("Is Ball:", lib.isBall());
        telemetry.addData("Ball count:", lib.getBallCount());
        telemetry.addData("Ball Color:", lib.getBallColor());

        telemetry.update();
    }
    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


}
