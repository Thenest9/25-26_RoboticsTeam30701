package org.firstinspires.ftc.teamcode.TestingPossibleAutonsDarv;

//------------------------GETTING IMPORTS------------------------//
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//------------------------FINISH GETTING IMPORTS------------------------//


/*                                  _________________
                    |           |               |
                    |           |               |
                    |           |               |
                    |-----------|               |
                    |           |               |
                    |           |               |
                    |           |       ________|_______

*/

public class BaseBlueStartTest extends OpMode
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

        //The following are used to tell if the robot is currently doing something
        public boolean isShooting = false;

        //Storing which balls have yet to be intaked by the robot
        public boolean[] availableBalls = {true, true, true};

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
            follower.setStartingPose(startPose);
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


                            //Once it is facing the motif, then it gets the motif and looks at the goal
                            if(follower.atPose())
                            {
                                motif = getMotif();
                                driveStates = DRIVESTATES.SHOOT;
                            }
                        }

                        else if(driveStates == DRIVESTATES.SHOOT)
                        {

                            //Where to move the robot

                            if(follower.atPose())
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
                                else if(availableBalls[2])
                                {
                                    driveStates = DRIVESTATES.BOT;
                                }
                            }


                        }

                        else if(driveStates == DRIVESTATES.TOP)
                        {
                            //Where to move the robot

                            if(follower.atPose())
                            {
                                availableBalls[0] = false;
                                currentState = STATES.INTAKE;
                            }
                        }

                        else if(driveStates == DRIVESTATES.MIDDLE)
                        {
                            //Where to move the robot

                            if(follower.atPose())
                            {
                                availableBalls[1] = false;
                                currentState = STATES.INTAKE;
                            }
                        }

                        else if(driveStates == DRIVESTATES.BOT)
                        {
                            //Where to move the robot

                            if(follower.atPose())
                            {
                                availableBalls[2] = false;
                                currentState = STATES.INTAKE;
                            }
                        }

                        else if(driveStates == DRIVESTATES.END)
                        {
                            //Where to move the robot

                            if(follower.atPose())
                            {
                                currentState = STATES.DONE;
                            }
                        }
                    }
                    break;


                //Turns the robot to shoot three balls, insures the ramp is up, sets the gate to
                // close, starts the flywheels, and turn the carousel
                case SHOOT:
                    //If we want to shoot and we currently are not shooting
                    if(!isShooting)
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
                    else if(shootTime.getElapsedTimeSeconds() > 2.67 && isShooting)
                    {
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
                    break;

                case DONE:


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

        public static String getMotif()
        {

        }

        public static void orderBalls()
        {

        }

        public static void shootThree()
        {

        }

        public static void rampUp()
        {

        }

        public static void rampDown()
        {

        }

        public static void gateOpen()
        {

        }

        public static void gateClose()
        {

        }

        public static void ifBallPresent()
        {

        }

        public static void driveRobot(Path path)
        {

        }

    //------------------------FINISH DECLARING METHODS------------------------//
}