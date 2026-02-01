package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawDebug;

import static java.lang.Thread.sleep;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import org.firstinspires.ftc.teamcode.Library;

@Autonomous(name = "Base Start Blue", group = "Autonomous") // Panels
public class BaseStartBluePedro extends OpMode
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
    private Timer pathTimer, actionTimer, opmodeTimer , motifTimer;
    private int pathState; // Current autonomous path state (state machine)
    private final Pose startPose= new Pose(20.688, 122.492, Math.toRadians(140));
    public Path shootingPose;
    public PathChain collect11, collect12, collect1SP, collect21, collect22, collect23, collect2SP;
    public PathChain path9,path10;

    public String currMotif="";


    public void buildPaths()
    {
        shootingPose = new Path(new BezierLine(new Pose(20.688, 122.492), new Pose(44.795,98.385)));
        shootingPose.setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(75));

        path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.795, 98.385),
                                new Pose(44.695, 98.385)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(150))
                .build();

        collect11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.695, 98.385),
                                new Pose(44.695, 81.364)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                .build();

        collect12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.695, 81.364),
                                new Pose(16.000, 81.159)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        collect1SP = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 81.159),

                                new Pose(44.695, 98.385)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

                .build();

        collect21 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.795, 98.385),

                                new Pose(44.795, 57.354)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))

                .build();

        collect22 = follower.pathBuilder().addPath(new BezierLine(new Pose(44.795, 57.354), new Pose(8.000, 57.354))
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

//        collect23 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(8.000, 57.354),
//
//                                new Pose(25.79516358463727, 57.354)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
//
//                .build();

        collect2SP = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(8.000, 57.354),
                                new Pose(47.447, 57.868),
                                new Pose(44.795, 98.385)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))

                .build();

        path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.795, 98.385),

                                new Pose(20.000, 72.717)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(270))

                .build();
    }


    public void autonomousPathUpdate()
    {

        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState)
        {
            case 0:
                follower.followPath(shootingPose);
                motifTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy())
                {
                    if(motifTimer.getElapsedTimeSeconds()>1 && currMotif.isEmpty())
                    {
                        currMotif = lib.getMotif();
                    }
                    follower.followPath(path10, true);
//                    while(motifTimer.getElapsedTimeSeconds() < 0.5)
//                    {
//                    }
//                    lib.orderBalls(currMotif, "ppg");

                    lib.shootThree(1280);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && !lib.isShooting && pathTimer.getElapsedTimeSeconds()>4)
                {
                    lib.rampDown();
                    follower.followPath(collect11, true);//infront of row 1 to intake
                    lib.IntakeStart();//starts intake
                    setPathState(3);//moves onto next path
                }
                break;
            case 3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1)//checks if it stopped following previous path, checks if its been at leat 0.5 seconds
                {
                    follower.followPath(collect12, 0.5, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() && !lib.isIntaking && pathTimer.getElapsedTimeSeconds()>5)
                {
                    actionTimer.resetTimer();
                    lib.rampDown();
                    lib.rampUp();
//                    while(actionTimer.getElapsedTimeSeconds()<0.5)
//                    {
//                        intake.setPower(-0.7);
//                    }
//                    if(actionTimer.getElapsedTimeSeconds()==0.53)
//                    {
//                        lib.rampUp();
//                    }
//                    intake.setPower(0.0);
//                    lib.runTogether(//TEST
//                            ()-> lib.orderBalls(currMotif, "gpp"),
//                    );

                        follower.followPath(collect1SP, true);
                    setPathState(5);
                }

                break;
            case 5://will end auto, set drivers up to intake, ramop will be down when this ends.
            if(!follower.isBusy()) {//if it stopped moving on the path shoot
                lib.shootThree(1367);
            }
//                if(!follower.isBusy()&& !lib.isShooting && pathTimer.getElapsedTimeSeconds()>3)
//                {
//                    lib.rampDown();
//                    follower.followPath(collect21, true);
//                    setPathState(6);
//                }
                break;
//            case 6:
//                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1)
//                {
//                    follower.followPath(collect22,true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if(!follower.isBusy())
//                {
//                    follower.followPath(collect2SP,true);
//                    lib.shootThree(1267);
//                    setPathState(8);
//                }
//                break;
//            case 8:
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
        motifTimer = new Timer();

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
//        if(currMotif.isEmpty())
//        {
//            currMotif=lib.getMotif();
//        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Shoot timer: ", lib.getShootTimer());
        telemetry.addData("isShooting:", lib.isShooting);
        telemetry.addData("intake timer: ", lib.getIntakeTimer());
        telemetry.addData("Carousel timer: ", lib.getCarTimer());
        telemetry.addData("isIntaking:", lib.isIntaking);
        telemetry.addData("Action Timer: ", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("Motif:", lib.getMotif());
        telemetry.addData("Motif", currMotif);
        telemetry.addData("Bottom", touchSensorBot.getState());
        telemetry.addData("Is Ball:", lib.isBall());
        telemetry.addData("Ball count:", lib.getBallCount());
        telemetry.addData("Ball Color:", lib.getBallColor());
        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.addData("Green: ", colorSensor.green());

        telemetry.update();
    }

    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}