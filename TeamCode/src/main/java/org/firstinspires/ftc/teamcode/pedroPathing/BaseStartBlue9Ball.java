package org.firstinspires.ftc.teamcode.pedroPathing;

//------------------------GETTING IMPORTS------------------------//
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

public class BaseStartBlue9Ball extends OpMode
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

    //Making the CRServo for the carousel
    CRServo carousel;

    //Making the touch sensors for the ramp so we know when it is all the way up or down
    DigitalChannel touchSensorBot;
    DigitalChannel touchSensorTop;

    //Making the gate to have the gate open or closed depending on intake or output
    Servo gate;

    //Tuning for the flywheels so they can get to speed faster
    double shooterP = 48.72995;
    double shooterI = 0;
    double shooterD = 0;
    double shooterF = 13.13319;

    //Used in pedropathing to move the robot along a certain path
    private Follower follower;

    //The position for where the robot starts on the field
    private final Pose startPose = new Pose(20.688, 122.492, Math.toRadians(140));

    //The position for where the robot shoots on the field
    public Path shootingPose = new Path(new BezierLine(new Pose(20.688, 122.492), new Pose(44.795,98.385)));

    //No idea what it does
    public PathChain collect11, collect12, collect1SP, collect21, collect22, collect2SP, path9, path10;
    //End here


    //The value of the motif
    public String motif;

    //The balls currently in the carousel
    public String order = "ppg";

    //A list of options the robot can be doing
    enum STATES {DRIVE, SHOOT, INTAKE, DONE}

    //What the robot is currently doing
    STATES currentState = STATES.DRIVE;

    //A list of places the robot could be at
    enum DRIVESTATES {START,SHOOT, MOTIF, TOP, MIDDLE, BOT, END}

    //What the robot is going to start off doing
    DRIVESTATES driveStates = DRIVESTATES.START;

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


    //All the way points the robot is going to go, each name tells its own one
    Pose start = new Pose(121, 126, Math.toRadians(37));

    Pose motifPos = new Pose(53, 90, Math.toRadians(85));

    Pose shoot = new Pose(53, 90, Math.toRadians(144));

    Pose topIntakeStart = new Pose(42, 82, Math.toRadians(180));

    Pose topIntakeEnd = new Pose(16, 82, Math.toRadians(180));

    Pose midIntakeStart = new Pose(42,58,Math.toRadians(180));

    Pose midIntakeEnd = new Pose(16,58,Math.toRadians(180));

    Pose gatePos = new Pose(19,70,Math.toRadians(0));

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





    //------------------------DECLARING HARDWARE------------------------//
    public void init()
    {
        // Initialize the right wheel of the fly wheel
        outputRight = hardwareMap.get(DcMotorEx.class, "RightOutput");
        outputRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outputRight.setDirection(DcMotorSimple.Direction.REVERSE);
        outputRight.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);

        // Initialize the left wheel of the fly wheel
        outputLeft = hardwareMap.get(DcMotorEx.class, "LeftOutput");
        outputLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        //Initialize the touch sensor at the top of the ramp
        touchSensorTop = hardwareMap.get(DigitalChannel.class, "touchSensorTop");
        touchSensorTop.setMode(DigitalChannel.Mode.INPUT);

        //Initialize the touch sensor at the top of the ramp
        touchSensorBot = hardwareMap.get(DigitalChannel.class, "touchSensorDown");
        touchSensorBot.setMode(DigitalChannel.Mode.INPUT);

        //Creating the follower object so the robot knows to follow certain paths
        follower = Constants.createFollower((hardwareMap));
        follower.setStartingPose(start);
    }
    //------------------------FINISH DECLARING HARDWARE------------------------//



    /*                                  _________________
                    |           |               |
                    |           |               |
                    |           |               |
                    |-----------|               |
                    |           |               |
                    |           |               |
                    |           |       ________|_______

    */

    //
    public void start()
    {
        gateClose();
        rampUp();
        carousel.setPower(0);
        outputLeft.setVelocity(0);
        outputRight.setVelocity(0);
    }



    //------------------------RUNNING CODE-----------------------//
    public void loop()
    {
        follower.update();

        //Switches to the case currentState is on
        switch (currentState)
        {
            //Drives the robot to the next position it needs to go
            case DRIVE:
                if(!follower.isBusy())
                {
                    if(driveStates == DRIVESTATES.MOTIF)
                    {
                        //Where to move the robot
                        follower.followPath(new Path(new BezierLine(follower.getPose(), motifPos)));


                        //Once it is facing the motif, then it gets the motif and looks at the goal
                        if(follower.atPose(motifPos, 4, 4))
                        {
                            motif = getMotif();
                            driveStates = DRIVESTATES.SHOOT;

                            startSorting();
                        }
                    }

                    else if(driveStates == DRIVESTATES.SHOOT)
                    {

                        //Where to move the robot
                        follower.followPath(new Path(new BezierLine(follower.getPose(), shoot)));


                        if(follower.atPose(shoot, 4, 4))
                        {
                            currentState = STATES.SHOOT;

                            if(availableBalls[0])
                            {
                                driveStates = DRIVESTATES.TOP;
                            }
                            else if(availableBalls[1])
                            {
                                driveStates = DRIVESTATES.MIDDLE;
                            }
                            else
                            {
                                driveStates = DRIVESTATES.END;
                            }
                        }


                    }

                    else if(driveStates == DRIVESTATES.TOP)
                    {
                        //Where to move the robot
                        follower.followPath(new Path(new BezierLine(follower.getPose(), topIntakeStart)));


                        if(follower.atPose(topIntakeStart, 4, 4))
                        {
                            availableBalls[0] = false;
                            currentState = STATES.INTAKE;
                            order = "ppg";
                        }
                    }

                    else if(driveStates == DRIVESTATES.MIDDLE)
                    {
                        //Where to move the robot
                        follower.followPath(new Path(new BezierLine(follower.getPose(), midIntakeStart)));


                        if(follower.atPose(midIntakeStart, 4, 4))
                        {
                            availableBalls[1] = false;
                            currentState = STATES.INTAKE;
                            order = "pgp";
                        }
                    }

                    else if(driveStates == DRIVESTATES.END)
                    {
                        //Where to move the robot
                        follower.followPath(new Path(new BezierLine(follower.getPose(), gatePos)));

                        if(follower.atPose(gatePos, 4, 4))
                        {
                            currentState = STATES.DONE;
                        }
                    }
                }
                break;


            //Turns the robot to shoot three balls, insures the ramp is up, sets the gate to
            // close, starts the flywheels, and turn the carousel
            case SHOOT:

                if(!sortingDone)
                {
                    break;
                }

                //If we want to shoot and we currently are not shooting
                if (!isShooting)
                {
                    //Put the ramp up
                    rampUp();

                    //Restart the timer
                    shootTime.resetTimer();

                    //Start the flywheels
                    outputLeft.setVelocity(1367);
                    outputRight.setVelocity(1367);

                    //Start spinning the carousel
                    carousel.setPower(-1);

                    //Tell the code that we are currently shooting
                    isShooting = true;
                }

                //Once all balls have been shot
                else if (shootTime.getElapsedTimeSeconds() > 2.67 && isShooting) {
                    //Turn off the flywheels
                    outputLeft.setVelocity(0);
                    outputRight.setVelocity(0);

                    //Turn off the carousel
                    carousel.setPower(0);

                    //Let the code know that we arent shooting
                    isShooting = false;

                    //Tell the code to start driving
                    currentState = STATES.DRIVE;
                }
                break;

            //Starts moving forward, turns on the intake, and checks weather to see if a balls is
            //occupying the current section, if so it turns the carousel, if not then it does nothing
            case INTAKE:

                //Where to move the robot
                Pose correctPose;

                if(follower.getPose().getY() == topIntakeStart.getY())
                {
                    correctPose = topIntakeEnd;
                }
                else
                {
                    correctPose = midIntakeEnd;
                }

                if(!isIntaking)
                {
                    rampDown();

                    //Turns on the intake
                    intake.setPower(1);

                    //Lets the code know that intake has been started
                    isIntaking = true;

                    follower.followPath(new Path(new BezierLine(follower.getPose(), correctPose)));
                }

                //If the robot is intaking and has reached its final destination
                else if(isIntaking && follower.atPose(correctPose, 4, 4))
                {
                    //turn off intake
                    intake.setPower(0);

                    //Tell the code that intake has stoped
                    isIntaking = false;

                    //Set the current state to drive so you can go to the next driving position
                    currentState = STATES.DRIVE;

                    //Set the driving state to shoot
                    driveStates = DRIVESTATES.SHOOT;

                    startSorting();
                }

                else
                {
                    if(isBall())
                    {
                        carousel.setPower(0.25);
                        spindexTime.resetTimer();
                        while(spindexTime.getElapsedTimeSeconds() < 1)
                        {

                        }
                        carousel.setPower(0);
                    }
                }

                break;

            case DONE:
                telemetry.addData("Robot State: ", "DONE");
                telemetry.update();
        }
    }
    //------------------------FINISH RUNNING CODE-----------------------//


    /*                                  _________________
                    |           |               |
                    |           |               |
                    |           |               |
                    |-----------|               |
                    |           |               |
                    |           |               |
                    |           |       ________|_______

    */



    //------------------------DECLARING METHODS------------------------//


    //Gets the value of the Motif and returns it to the call
    public String getMotif()
    {
        //Makes the String object which will be returned at the end
        String motif = "";

        //Switches the limelight to the pipeline to see the motif
        limelight.pipelineSwitch(7);
        LLResult result = limelight.getLatestResult();

        //If the motif sees something and it isnt empty
        if (result.isValid() && !result.getFiducialResults().isEmpty())
        {
            //Gets what ID number is displayed on the April Tag (23, 22, or 21)
            LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
            int detectedTagId = fiducial.getFiducialId();
            telemetry.addData("AprilTag ID", detectedTagId);

            //If the tag the limelight sees is 23, the code is PPG
            if (detectedTagId == 23)
            {
                motif = "ppg";
                telemetry.addData("Color:", "Purple, Purple, Green");
            }

            //If the tag the limelight sees is 22, the code is PGP
            else if(detectedTagId == 22)
            {
                motif = "pgp";
                telemetry.addData("Color:", "Purple, Green, Purple");
            }

            //If the tag the limelight sees is 21, the code is GPP
            else if(detectedTagId == 21)
            {
                motif = "gpp";
                telemetry.addData("Color:", "Green, Purple, Purple");
            }
        }

        telemetry.update();
        return motif;
    }


    //Orders the balls depending on the order passed through and the value of the Motif
    public void orderBalls(String motif, String order)
    {
        //Moves the gate to the front area of the carousel, the values are absolute
        gate.setPosition(0.67);

        telemetry.addData("Motif: ", motif);
        telemetry.addData("Order: ", order);
        telemetry.update();


        carousel.setPower(0);
        spindexTime.resetTimer();
        while(spindexTime.getElapsedTimeSeconds() < 0.5)
        {

        }

        while(!motif.equals(order))
        {
            //Spins the carousel to the next section
            carousel.setPower(0.25);
            spindexTime.resetTimer();
            while(spindexTime.getElapsedTimeSeconds() < 1)
            {

            }

            //Stops the carousel from spinning
            carousel.setPower(0);

            //Move the last item in the string to the front
            order = order.substring(2) + order.substring(0,2);
        }

        telemetry.addData("Motif: ", motif);
        telemetry.addData("Order: ", order);
        telemetry.update();

        gate.setPosition(0.95);
    }


    //A method to put the ramp up
    public void rampUp()
    {
        //While the sensor on the top is not getting clicked
        while(touchSensorTop.getState())
        {
            //Set the ramp to go up, and the intake to go inwards
            ramp.setPower(0.5);
            intake.setPower(1);
        }
        //Turn the ramp and the intake off
        ramp.setPower(0);
        intake.setPower(0);
    }

    //A method to put the ramp down
    public void rampDown()
    {
        //While the sensor on the top is not getting clicked
        while(touchSensorBot.getState())
        {
            //Set the ramp to go down, and the intake to go outwards
            ramp.setPower(-0.5);
            intake.setPower(-1);
        }
        //Turn the ramp and the intake off
        ramp.setPower(0.0);
        intake.setPower(0.0);

    }

    //Opens the gate to be able to spindex
    public void gateOpen()
    {
        gate.setPosition(0.67);
    }

    //Closes the gate to be able to shoot
    public void gateClose()
    {
        gate.setPosition(0.95);
    }


    //Checks if there is a ball in front of the carousel
    public boolean isBall()
    {
        //If a ball is there and the count is less than 2
        if(colorSensor.red()>40 && balls < 2)
        {
            //Increase the balls count by 1
            balls++;

            //Lets the main program know that a ball was seen
            return true;
        }

        //If no ball is seen in front of the carousel
        else
        {
            //Lets the main program know that a ball was not seen
            return false;
        }
    }

    public void startSorting()
    {
        if (OrderBalls != null && OrderBalls.isAlive()) return;

        sortingDone = false;
        OrderBalls = new Thread(() -> {
            orderBalls(motif, order);
            sortingDone = true;
        });
        OrderBalls.start();
    }


    //------------------------FINISH DECLARING METHODS------------------------//
}