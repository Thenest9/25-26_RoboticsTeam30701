package org.firstinspires.ftc.teamcode.pedroPathing;

//------------------------GETTING IMPORTS------------------------//
import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawDebug;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

//------------------------FINISH GETTING IMPORTS------------------------//


/*                                      _________________
                    |           |               |
                    |           |               |
                    |           |               |
                    |-----------|               |
                    |           |               |
                    |           |               |
                    |           |       ________|_______

*/

@Autonomous(name = "BaseStartRed9Ball", group = "Autonomous")
public class BaseStartRed9Ball extends OpMode
{
    //------------------------DEFINING VARIABLES------------------------//

    //The camera at the top of the robot used to see the april tags
    Limelight3A limelight;

    //Making the DcMotors for the tires, the ramp, and the intake
    DcMotor FrontLeft, FrontRight, RearLeft, RearRight, intake, ramp;

    //Making the DcMotorEx for the fly wheels
    DcMotorEx outputRight, outputLeft;

    //Making the colorSensor to be able to check the color of the balls
    ColorSensor colorSensor;

    //Making the CRServo for carousel spindexing
    CRServo carousel;

    //Making the touch sensors for the ramp so we know when it is all the way up or down
    DigitalChannel touchSensorBot;
    DigitalChannel touchSensorTop;

    DigitalChannel magSwitch;

    //Making the gate to have the gate open or closed depending on intake or output
    Servo gate;

    //Tuning for the flywheels so they can get to speed faster
    double shooterP = 48.72995;
    double shooterI = 0;
    double shooterD = 0;
    double shooterF = 13.13319;

    //The value of the motif
    public String motif;

    //The balls currently in the carousel
    public String order = "ppg";

    //Timer so we can shoot the balls in the carousel for only 2.67 seconds
    public Timer shootTime = new Timer();

    //Timer so we know the times for carousel movement during spindexing
    public Timer spindexTime = new Timer();

    //The following are used to tell if the robot is currently doing something
    public boolean isShooting = false;

    //Storing which balls have yet to be intaked by the robot
    public boolean[] availableBalls = {true, true, true};

    //Int to know how many balls are in the carousel
    public int balls = 3;

    //Tells the robot wether or not we are intaking
    boolean isIntaking = false;

    public Thread OrderBalls;

    public boolean sortingDone = false;


    //------------------------FINISH DEFINING VARIABLES------------------------//


    /*                                  _________________
                    |           |               |
                    |           |               |
                    |           |               |
                    |-----------|               |
                    |           |               |
                    |           |               |
                    |           |       ________|_______

    */

    LibraryPedro lib;

    //Used in pedropathing to move the robot along a certain path
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer , motifTimer;
    private int pathState; // Current autonomous path state (state machine)
    private final Pose startPose = new Pose(124.5, 122, Math.toRadians(36)); //The position for where the robot starts on the field

    //Declaration of Paths
    public PathChain getMotifPose;
    public PathChain firstShot;
    public PathChain moveFirstRow;
    public PathChain collectFirstRow;
    public PathChain secondShot;
    public PathChain moveSecondRow;
    public PathChain collectSecondRow;
    public PathChain thirdShot;
    public PathChain EndPosition;

    public String currMotif = ""; // Motif

    public void buildPaths()
    {

        getMotifPose = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(124.500, 121.000),
                                new Pose(96.000, 95.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(115))
                .build();

        firstShot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(96, 95),
                                new Pose(105, 105)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(45))
                .build();

        moveFirstRow = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(105, 105),

                                new Pose(96.000, 61)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();

        collectFirstRow = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.000, 61),

                                new Pose(125.000, 61)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        secondShot = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(125.000, 61),
                                new Pose(96.000, 56.012),
                                new Pose(105, 105)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        moveSecondRow = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(105, 105),

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
        EndPosition = collectFirstRow = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.000, 95.5),

                                new Pose(125.000, 61)
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
            case 0://Moves back and looks at Motif
                follower.followPath(getMotifPose);
                motifTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.5)
                {
                    //Order balls, then case 2, shoot, then case 3, stop and move one
                    lib.orderBalls(currMotif, "ppg");
                    follower.followPath(firstShot);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy())
                {
                    lib.shootThree(1100);
                    setPathState(3);
                }

                break;

            case 3://move to be infront of balls
                if(!follower.isBusy() && !lib.isShooting && pathTimer.getElapsedTimeSeconds() > 3)
                {
                    while(magSwitch.getState())
                    {
                        carousel.setPower(0.2);
                    }
                    carousel.setPower(0);
                    lib.rampDown();
                    follower.followPath(moveFirstRow,1, true);//infront of row 1 to intake
                    lib.IntakeStart();//starts intake
                    setPathState(4);//moves onto next path
                }
                break;
            case 4:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2)//checks if it stopped following previous path, checks if its been at leat 0.5 seconds
                {
                    follower.followPath(collectFirstRow, 0.5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() && !lib.isIntaking && pathTimer.getElapsedTimeSeconds() > 5)
                {
                    actionTimer.resetTimer();
                    lib.rampUp();
                    lib.orderBalls(currMotif, "pgp");
                    follower.followPath(secondShot, true);

                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy())
                {
                    lib.shootThree(1150);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy() && !lib.isShooting && pathTimer.getElapsedTimeSeconds() > 3)
                {
                    lib.rampDown();
                    follower.followPath(moveSecondRow, 1, true);
                    lib.IntakeStart();
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2)//checks if it stopped following previous path, checks if its been at leat 0.5 seconds
                {
                    follower.followPath(collectSecondRow, 0.5, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy())
                {
                    lib.orderBalls(currMotif, "gpp");
                    follower.followPath(thirdShot, 1, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy())
                {
                    lib.shootThree(1150);
                    setPathState(11);
                }
            break;
            case 11:
                if(!lib.isShooting)
                {
                    follower.followPath(EndPosition);
                }
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


        touchSensorBot = hardwareMap.get(DigitalChannel.class, "touchSensorBot");
        touchSensorBot.setMode(DigitalChannel.Mode.INPUT);


        magSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch");
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
            if(lib.isBall()) {
                lib.carouselStart();
            }
            carousel.setPower(0);
        }
        if(currMotif.equals(""))
        {
            currMotif = lib.getMotif();
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
        telemetry.addData("GetMotif FUNCTION:", lib.getMotif());
        telemetry.addData("Motif", currMotif);
        telemetry.addData("Bottom", touchSensorBot.getState());
        telemetry.addData("Is Ball:", lib.isBall());
        telemetry.addData("Ball count:", lib.getBallCount());
        telemetry.addData("Ball Color:", lib.getBallColor());
        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.addData("Green: ", colorSensor.green());
        telemetry.addData("still following?", follower.isBusy());
        telemetry.addData("Carousel Power: ", carousel.getPower());
        telemetry.addData("IsOrdering: ", lib.getIsOrdering());

        telemetry.update();
    }

    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
