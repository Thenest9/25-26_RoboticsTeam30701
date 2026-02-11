package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawDebug;

import static java.lang.Thread.sleep;

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

@Autonomous(name = "BaseStartRed6Ball", group = "Autonomous")
public class BaseStartRed6Ball extends OpMode
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
    private int step;
    private final Pose startPose= new Pose(121.3658536585366, 126.82926829268294, Math.toRadians(36.5));//Change to your starting positon
    public Path Path1;//First Path MUST BE A PATH NOT PATH CHAIN
    public PathChain Path2, Path3, Path4, Path5, Path6;//rename to your paths
    public String currMotif="";


    public void buildPaths()//TODO copy all paths here (Make sure you change your first path to match the structure
    {
        Path1 = new Path (
                new BezierLine(
                        new Pose(121.366, 126.829),

                        new Pose(96.976, 103.317)
                )
        );
        Path1.setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(46));

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.976, 103.317),

                                new Pose(96.976, 103.317)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(36.5))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.976, 103.317),

                                new Pose(97.293, 84.707)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(97.293, 84.707),

                                new Pose(128.195, 84.829)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.195, 84.829),

                                new Pose(96.829, 103.244)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.829, 103.244),

                                new Pose(125.122, 73.293)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(270))

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
                follower.followPath(Path1);
                motifTimer.resetTimer();
                setPathState(1);
                break;

            case 1://Next path case
                if(motifTimer.getElapsedTimeSeconds()>1 && currMotif.isEmpty())
                {
                    lib.rampUp();
                    currMotif = lib.getMotif();
                }
                if(!follower.isBusy())
                {
                    follower.followPath(Path6, true);
                    // shoots 3 balls
                    lib.shootThree(1280);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy())
                {
                    follower.followPath(Path2, true);
                    // starts intake
                    lib.rampDown();
                    lib.IntakeStart();
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy())
                {
                    // intake finished
                    follower.followPath(Path3, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy())
                {
                    follower.followPath(Path4, true);
                    lib.shootThree(1280);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy())
                {
                    follower.followPath(Path5, true);
                }
                break;
        }
    }
    public void setPathState(int pState) //will reset PathTimer and change path states
    {
        pathState = pState;
        pathTimer.resetTimer();
//        step=0;
    }

    @Override
    public void init()//Leave this mostly alone
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


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer =  new Timer();
        opmodeTimer.resetTimer();
        motifTimer = new Timer();

        follower = Constants.createFollower((hardwareMap));
        buildPaths();
        follower.setStartingPose(startPose);//TODO Change with your start Pose

        lib = new LibraryPedro(outputRight, outputLeft, carousel, telemetry, limelight, intake, ramp, gate, colorSensor, touchSensorTop, touchSensorBot);

    }


    @Override
    public void loop()//runs about 50 times a second
    {
        follower.update(); // Update Pedro Pathing
        lib.updateShoot();//Updates Shooting
        lib.finishIntake();//Updates Intake
        if(lib.isIntaking)//
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