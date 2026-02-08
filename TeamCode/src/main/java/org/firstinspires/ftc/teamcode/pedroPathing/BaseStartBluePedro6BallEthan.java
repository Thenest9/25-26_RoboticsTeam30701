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

@Autonomous(name = "Base Start Blue 6 ball sorted", group = "Autonomous") // Panels
public class BaseStartBluePedro6BallEthan extends OpMode
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
    DigitalChannel magSwitch;

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
    private int step;
    private final Pose startPose= new Pose(20.688, 122.492, Math.toRadians(145));
    public Path motifPose;
    public PathChain collect11, collect12, collect1SP;
    public PathChain path9,shootingPose;

    public String currMotif="";


    public void buildPaths()
    {
        motifPose = new Path(new BezierLine(new Pose(20.688, 122.492), new Pose(44.795,98.385)));
        motifPose.setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(75));

        shootingPose = follower.pathBuilder().addPath(
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
                                new Pose(18.00, 81.159)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        collect1SP = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.000, 81.159),

                                new Pose(44.695, 98.385)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

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
            case 0://Moves back and looks at Motif
                follower.followPath(motifPose);
                motifTimer.resetTimer();
                setPathState(1);
                break;
            case 1://path 10 case,
                switch (step)
                {
                    case 0:
                        if(motifTimer.getElapsedTimeSeconds()>1 && currMotif.isEmpty())
                        {
                            currMotif = lib.getMotif();
                        }
                        if(!follower.isBusy()) {
                            follower.followPath(shootingPose, true);
                            step = 1;
                        }
                        break;
                    case 1:
                        if(!follower.isBusy())
                        {
                            //Order balls, then case 2, shoot, then case 3, stop and move one
                            lib.shootThree(1280);
                            step = 2;
                        }
                        break;
                    case 2:
                        if(!lib.isShooting)
                        {
                            step=0;
                            setPathState(2);
                        }
                        break;
                }
                break;
            case 2://move to be infront of balls
                if(!follower.isBusy() && !lib.isShooting && pathTimer.getElapsedTimeSeconds()>3)
                {
                    lib.rampDown();
                    follower.followPath(collect11, true);//infront of row 1 to intake
                    lib.IntakeStart();//starts intake
                    setPathState(3);//moves onto next path
                }
                break;
            case 3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2 )//checks if it stopped following previous path, checks if its been at leat 0.5 seconds
                {
                    follower.followPath(collect12, 0.8, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() && !lib.isIntaking && pathTimer.getElapsedTimeSeconds()>5)
                {
                    actionTimer.resetTimer();
                    lib.rampDown();
                    lib.rampUp();

                    lib.orderBalls(currMotif, "gpp");
                    follower.followPath(collect1SP, true);
                    setPathState(5);
                }

                break;
            case 5:
                if(!follower.isBusy() && lib.isDoneSpindexing)
                {
                    lib.shootThree(1300);
                    setPathState(6);
                }
                break;
            case 6:
                if(!lib.isShooting)
                {
                    follower.followPath(path9, true);
                    setPathState(6);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
//        step=0;
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


        magSwitch = hardwareMap.get(DigitalChannel.class, "magneticLimitSwitch");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);
        // Initialize the library class through a lib object

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer =  new Timer();
        opmodeTimer.resetTimer();
        motifTimer = new Timer();

        follower = Constants.createFollower((hardwareMap));
        buildPaths();
        follower.setStartingPose(startPose);

        lib = new LibraryPedro(outputRight, outputLeft, carousel, telemetry, limelight, intake, ramp, gate, colorSensor, touchSensorTop, touchSensorBot, magSwitch);

    }


    @Override
    public void loop()//runs about 50 times a second
    {
        follower.update(); // Update Pedro Pathing
        lib.updateShoot();
        lib.finishIntake();
        if(lib.isIntaking)
        {
            if(lib.isBall())
            {
                while(magSwitch.getState())
                {
                    carousel.setPower(0.5);
                }
                carousel.setPower(0);
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
        telemetry.addData("still following?", follower.isBusy());

        telemetry.update();
    }

    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}