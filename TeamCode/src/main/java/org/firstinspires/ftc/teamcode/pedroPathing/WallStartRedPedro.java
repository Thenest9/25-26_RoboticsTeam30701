package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
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

@Autonomous(name = "Wall Start Red", group = "Autonomous") // Panels
public class WallStartRedPedro extends OpMode
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
    public Path ShootingSpot;
    private final Pose startingPose= new Pose(80.6, 7.6, Math.toRadians(90));
    public PathChain BottomRow , CollectTheBalls , ShootingSpot2 , InfrontofMiddleBalls;
    public PathChain path9,path10;

    public String currMotif;


    public void buildPaths()
    {
        ShootingSpot = new Path (
                        new BezierLine(
                                new Pose(80.600, 7.600),

                                new Pose(83.558, 83.026)
                        )
                );
        ShootingSpot.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45));


        BottomRow = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(83.558, 83.026),

                            new Pose(102.000, 37.500)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

            .build();

        CollectTheBalls = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(102.000, 37.500),

                                new Pose(130.000, 37.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        ShootingSpot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(130.000, 37.500),
                                new Pose(96.527, 28.441),
                                new Pose(83.558, 83.026)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        InfrontofMiddleBalls = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(83.558, 83.026),

                                new Pose(102.969, 61.771)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

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
                follower.followPath(ShootingSpot);
                currMotif = lib.getMotif();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy())
                {
                    follower.followPath(path10, true);
//                    lib.orderBalls(currMotif, "ppg");
                    lib.shootThree(1367);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && !lib.isShooting && pathTimer.getElapsedTimeSeconds()>4)
                {
                    lib.rampDown();
                    follower.followPath(BottomRow, true);//infront of row 1 to intake
                    lib.IntakeStart();//starts intake
                    setPathState(3);//moves onto next path
                }
                break;
            case 3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2)//checks if it stopped following previous path, checks if its been at leat 0.5 seconds
                {
                    follower.followPath(CollectTheBalls, 0.5, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() && !lib.isIntaking && pathTimer.getElapsedTimeSeconds()>3)
                {
                    lib.rampDown();
                    lib.rampUp();
                    actionTimer.resetTimer();
//                    lib.runTogether(//TEST
//                            ()-> lib.orderBalls(currMotif, "ppg"),
//                            ()->follower.followPath(ShootingSpot2, true)
//                    );
                    setPathState(5);
                }
                break;
            case 5://will end auto, set drivers up to intake, ramop will be down when this ends.
//                if(!follower.isBusy()&& !lib.isShooting && pathTimer.getElapsedTimeSeconds()>3)
//                {
                if(!follower.isBusy()) {
                    lib.shootThree(1367);
                }
                break;
//                    lib.rampDown();
//                    follower.followPath(InfrontofMiddleBalls, true);
//                    setPathState(6);
//                }
//                break;
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

        follower = Constants.createFollower((hardwareMap));
        buildPaths();
        follower.setStartingPose(startingPose);

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
        telemetry.addData("Motif", currMotif);
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