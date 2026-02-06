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

@Autonomous(name = "This will show on Driver Hub", group = "Autonomous")
public class Template extends OpMode
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
    private final Pose startPose= new Pose(20.688, 122.492, Math.toRadians(145));//Change to your starting positon
    public Path motifPose;//First Path MUST BE A PATH NOT PATH CHAIN
    public PathChain collect11, collect12, collect1SP;//rename to your paths
    public PathChain path9,shootingPose;

    public String currMotif="";


    public void buildPaths()//TODO copy all paths here (Make sure you change your first path to match the structure
    {
        motifPose = new Path(new BezierLine(new Pose(20.688, 122.492), new Pose(44.795, 98.385)));
        motifPose.setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(75));

        shootingPose = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.795, 98.385),
                                new Pose(44.695, 98.385)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(150))
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
            case 1://Next path case
                 if(motifTimer.getElapsedTimeSeconds()>1 && currMotif.isEmpty())
                 {
                     currMotif = lib.getMotif();
                 }
                 if(!follower.isBusy())
                 {
                     follower.followPath(shootingPose, true);
                     lib.shootThree(1280);
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