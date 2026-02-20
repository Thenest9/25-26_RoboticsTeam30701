package org.firstinspires.ftc.teamcode.pedroPathing;

//------------------------GETTING IMPORTS------------------------//
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

@Autonomous(name = "Base Start Blue 9 Ball")
public class BaseStartBlue9Ball extends OpMode {
    //------------------------DEFINING VARIABLES------------------------//

    //The camera at the top of the robot used to see the april tags
    private Limelight3A limelight;

    //Making the DcMotors for the tires, the ramp, and the intake
    private DcMotor FrontLeft, FrontRight, RearLeft, RearRight, intake, ramp;

    //Making the DcMotorEx for the fly wheels
    private DcMotorEx outputRight, outputLeft;

    //Making the colorSensor to be able to check the color of the balls
    private ColorSensor colorSensor;

    //Making the CRServo for the carousel
    private CRServo carousel;

    //Making the touch sensors for the ramp so we know when it is all the way up or down
    private DigitalChannel touchSensorBot;
    private DigitalChannel touchSensorTop;

    //Making the gate to have the gate open or closed depending on intake or output
    private Servo gate;

    //Making a variable for the magnetic limit switch
    private DigitalChannel magneticSwitch;

    //Used in pedropathing to move the robot along a certain path
    private Follower follower;

    //The value of the motif
    private String motif;

    //The balls currently in the carousel
    private String order = "ppg";

    //A list of options the robot can be doing
    private enum STATES {DRIVE, SHOOT, INTAKE, DONE}

    //What the robot is currently doing
    private STATES currentState = STATES.DRIVE;

    //A list of places the robot could be at
    private enum DRIVESTATES {SHOOT, MOTIF, TOP, MIDDLE, BOT, END}

    //What the robot is going to start off doing
    private DRIVESTATES driveStates = DRIVESTATES.MOTIF;

    //Timer so we can shoot the balls in the carousel for only 2.67 seconds
    private final Timer shootTime = new Timer();

    //Timer so we know the times for carousel movement during spindexing
    private final Timer spindexTime = new Timer();

    //The following are used to tell if the robot is currently doing something
    private boolean isShooting = false;

    //Storing which balls have yet to be intaked by the robot
    private final boolean[] availableBalls = {true, true, true};

    //Int to know how many balls are in the carousel
    private int balls = 3;

    //Tells the robot whether or not we are intaking
    private boolean isIntaking = false;

    //Tells the robot whether or not we are moving

    private boolean isMoving = false;


    //All the way points the robot is going to go, each name tells its own one
    private final Pose start = new Pose(20, 122, Math.toRadians(144));

    private final Pose motifPos = new Pose(53, 90, Math.toRadians(85));

    private final Pose shoot = new Pose(53, 90, Math.toRadians(144));

    private final Pose topIntakeStart = new Pose(42, 82, Math.toRadians(180));

    private final Pose topIntakeEnd = new Pose(16, 82, Math.toRadians(180));

    private final Pose midIntakeStart = new Pose(42, 58, Math.toRadians(180));

    private final Pose midIntakeEnd = new Pose(16, 58, Math.toRadians(180));

    private final Pose endPos = new Pose(19, 70, Math.toRadians(0));

    //A thread which handles the parallel computing part of sorting balls
    private Thread OrderBalls;

    //Lets the robot know if the sorting is done yet or not
    private boolean sortingDone = false;

    //The position the robot will be right before it has to shoot
    private Pose correctPose = motifPos;

    private String rampPos = "up";

    private boolean isRampMoving = false;

    private boolean isCarouselOn = false;


    //Paths the robot is gonna follow
    private final Path startToMotif = new Path(new BezierLine(start, motifPos));

    private Path correctPoseToShoot = new Path(new BezierLine(correctPose, shoot));

    private Path intakeStartToEnd;

    private final Path shootToTop = new Path(new BezierLine(shoot, topIntakeStart));

    private final Path shootToMid = new Path(new BezierLine(shoot, midIntakeStart));

    private final Path shootToEnd = new Path(new BezierLine(shoot, endPos));
    ;


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
    public void init() {
        //Tuning for the flywheels so they can get to speed faster
        final double shooterP = 48.72995;
        final double shooterI = 0;
        final double shooterD = 0;
        final double shooterF = 13.13319;

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

        //Initializing the magnetic limit switch for more precise carousel movements
        magneticSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch");
        magneticSwitch.setMode(DigitalChannel.Mode.INPUT);
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
    public void loop() {
        //Updates the pedropathing so even if you move it, it always stays on the position it needs to be
        follower.update();

        keepRampStill();

        //Switches to the case currentState is on
        switch (currentState) {
            //Drives the robot to the next position it needs to go
            case DRIVE:
                //If the robot needs to drive to the location of the motif
                if (driveStates == DRIVESTATES.MOTIF) {
                    //When the robot is not driving
                    if (!isMoving) {
                        //Moving the robot to face the motif
                        follower.followPath(startToMotif);

                        //Tells the robot that we are now moving
                        isMoving = true;
                    }

                    //Once it is facing the motif, then it gets the motif and looks at the goal
                    else if (follower.atPose(motifPos, 4, 4)) {
                        //Lets the robot know we stopped moving
                        isMoving = false;

                        //Gets the value of the motif
                        motif = getMotif();

                        //Lets the robot know to start shooting
                        driveStates = DRIVESTATES.SHOOT;

                        //Starts the shooting thread on the robot
                        startSorting();
                    }
                }

                //If the robot needs to drive to the location for it to shoot
                else if (driveStates == DRIVESTATES.SHOOT) {
                    //When the robot is not driving
                    if (!isMoving) {
                        //Where to move the robot
                        follower.followPath(correctPoseToShoot);

                        //Tells the robot that we are now moving
                        isMoving = true;
                    }

                    //If the robot is at the shooting position
                    else if (follower.atPose(shoot, 4, 4)) {
                        //Lets the robot know we stopped moving
                        isMoving = false;

                        //Tells the robot to know shoot
                        currentState = STATES.SHOOT;

                        //If the balls at the top have not been intaked
                        if (availableBalls[0]) {
                            //Tells the robot to drive to the top intake and set the correct
                            //position to the end of the intakes
                            driveStates = DRIVESTATES.TOP;
                            correctPose = topIntakeEnd;
                            correctPoseToShoot = new Path(new BezierLine(correctPose, shoot));
                            intakeStartToEnd = new Path(new BezierLine(topIntakeStart, topIntakeEnd));
                        }

                        //If the balls at the mid have not been intaked
                        else if (availableBalls[1]) {
                            //Tells the robot to drive to the mid intake and set the correct
                            //position to the end of the intakes
                            driveStates = DRIVESTATES.MIDDLE;
                            correctPose = midIntakeEnd;
                            correctPoseToShoot = new Path(new BezierLine(correctPose, shoot));
                            intakeStartToEnd = new Path(new BezierLine(midIntakeStart, midIntakeEnd));
                        }

                        //If both rows have been intaked
                        else {
                            //Tells the robot to park at the end spot
                            driveStates = DRIVESTATES.END;
                        }
                    }


                }

                //If the robot needs to drive to the location for it to start intaking the top row of balls
                else if (driveStates == DRIVESTATES.TOP) {
                    //When the robot is not driving
                    if (!isMoving) {
                        //Where to move the robot
                        follower.followPath(shootToTop);

                        //Tells the robot that we are now moving
                        isMoving = true;
                    }

                    //Once the robot is at the intaking position
                    if (follower.atPose(topIntakeStart, 4, 4)) {
                        //Lets the robot know we stopped moving
                        isMoving = false;

                        //Tells the robot that this row of balls wont be available for intake
                        availableBalls[0] = false;

                        //Tells the robot to start intaking
                        currentState = STATES.INTAKE;

                        //Stores the order of the balls in the carousel
                        order = "ppg";
                    }
                }

                //If the robot needs to drive to the location for it to start intaking the mid row of balls
                else if (driveStates == DRIVESTATES.MIDDLE) {
                    //When the robot is not driving
                    if (!isMoving) {
                        //Where to move the robot
                        follower.followPath(shootToMid);

                        //Tells the robot that we are now moving
                        isMoving = true;
                    }

                    //Once the robot is at the intaking position
                    if (follower.atPose(midIntakeStart, 4, 4)) {
                        //Lets the robot know we stopped moving
                        isMoving = false;

                        //Tells the robot that this row of balls wont be available for intake
                        availableBalls[1] = false;

                        //Tells the robot to start intaking
                        currentState = STATES.INTAKE;

                        //Stores the order of the balls in the carousel
                        order = "pgp";
                    }
                }

                //If the robot needs to drive to the location to park
                else if (driveStates == DRIVESTATES.END) {
                    //When the robot is not driving
                    if (!isMoving) {
                        //Where to move the robot
                        follower.followPath(shootToEnd);

                        //Tells the robot that we are now moving
                        isMoving = true;
                    }

                    //Once the robot is at the end position
                    if (follower.atPose(endPos, 4, 4)) {
                        //Lets the robot know we stopped moving
                        isMoving = false;

                        //Tells the robot that it is done doing things
                        currentState = STATES.DONE;
                    }
                }
                break;


            //Turns the robot to shoot three balls, insures the ramp is up, sets the gate to
            // close, starts the flywheels, and turn the carousel
            case SHOOT:

                //If the sorting isn't done,
                if (!sortingDone) {
                    break;
                }

                //If we want to shoot and we currently are not shooting
                if (!isShooting) {
                    //Puts the ramp up
                    rampUp();

                    if(!isRampMoving)
                    {
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
                //When the robot has not started intaking
                if (!isIntaking)
                {
                    //Puts the ramp down
                    rampDown();

                    if(!isRampMoving)
                    {
                        //Turns on the intake
                        intake.setPower(1);

                        //Lets the code know that intake has been started
                        isIntaking = true;

                        //Makes the robot go from its current position to the ending spot it needs to
                        follower.followPath(intakeStartToEnd);
                    }
                }

                //If the robot is intaking and has reached its final destination
                else if (follower.atPose(correctPose, 4, 4)) {
                    //turn off intake
                    intake.setPower(0);

                    //Tell the code that intake has stoped
                    isIntaking = false;

                    //Set the current state to drive so you can go to the next driving position
                    currentState = STATES.DRIVE;

                    //Set the driving state to shoot
                    driveStates = DRIVESTATES.SHOOT;

                    //Opens a thread which sorts the balls in the robot according to the motif
                    startSorting();
                }

                //If a ball is present, then it turns the carousel for one rotation
                else {
                    //If the color sensor sees a ball
                    if (isBall()) {
                        //Tells the robot to rotate the carousel by one section
                        rotateOneSection();
                    }
                }

                break;

            //Once all the tasks are done
            case DONE:
                //Puts on the control hub that we are done
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
    public String getMotif() {
        //Makes the String object which will be returned at the end
        String motif = "";

        //Switches the limelight to the pipeline to see the motif
        limelight.pipelineSwitch(7);
        LLResult result = limelight.getLatestResult();

        //If the motif sees something and it isnt empty
        if (result.isValid() && !result.getFiducialResults().isEmpty()) {
            //Gets what ID number is displayed on the April Tag (23, 22, or 21)
            LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
            int detectedTagId = fiducial.getFiducialId();
            telemetry.addData("AprilTag ID", detectedTagId);

            //If the tag the limelight sees is 23, the code is PPG
            if (detectedTagId == 23) {
                motif = "ppg";
                telemetry.addData("Color:", "Purple, Purple, Green");
            }

            //If the tag the limelight sees is 22, the code is PGP
            else if (detectedTagId == 22) {
                motif = "pgp";
                telemetry.addData("Color:", "Purple, Green, Purple");
            }

            //If the tag the limelight sees is 21, the code is GPP
            else if (detectedTagId == 21) {
                motif = "gpp";
                telemetry.addData("Color:", "Green, Purple, Purple");
            }
        }

        telemetry.update();
        return motif;
    }


    //Orders the balls depending on the order passed through and the value of the Motif
    public void orderBalls(String motif, String order) {
        //Moves the gate to the front area of the carousel, the values are absolute
        gateOpen();

        telemetry.addData("Motif: ", motif);
        telemetry.addData("Order: ", order);
        telemetry.update();

        while (!motif.equals(order)) {
            //Tells the robot to rotate the carousel by one section
            rotateOneSection();

            //Move the last item in the string to the front
            order = order.substring(2) + order.substring(0, 2);
        }

        telemetry.addData("Motif: ", motif);
        telemetry.addData("Order: ", order);
        telemetry.update();

        gateClose();
    }

    //Opens the gate to be able to spindex
    public void gateOpen() {
        gate.setPosition(0.67);
    }

    //Closes the gate to be able to shoot
    public void gateClose() {
        gate.setPosition(0.95);
    }


    //Checks if there is a ball in front of the carousel
    public boolean isBall() {
        //If a ball is there and the count is less than 2
        if (colorSensor.red() > 40 && balls < 2) {
            //Increase the balls count by 1
            balls++;

            //Lets the main program know that a ball was seen
            return true;
        }

        //If no ball is seen in front of the carousel
        else {
            //Lets the main program know that a ball was not seen
            return false;
        }
    }

    public void startSorting() {
        if (OrderBalls != null && OrderBalls.isAlive()) return;

        sortingDone = false;
        OrderBalls = new Thread(() ->
        {
            orderBalls(motif, order);
            sortingDone = true;
        });
        OrderBalls.start();
    }

    public void keepRampStill() {
        if (rampPos.equals("up"))
        {
            ramp.setPower(0.05);
        }

        else
        {
            ramp.setPower(-0.05);
        }
    }

    public void rampUp()
    {
        if (!isRampMoving)
        {
            isRampMoving = true;
        }

        ramp.setPower(0.5);

        if (!touchSensorTop.getState())
        {
            ramp.setPower(0);
            isRampMoving = false;
            rampPos = "up";
        }
    }

    public void rampDown()
    {
        if (!isRampMoving)
        {
            isRampMoving = true;
        }

        ramp.setPower(-0.5);

        if (!touchSensorBot.getState())
        {
            ramp.setPower(0);
            isRampMoving = false;
            rampPos = "down";
        }
    }

    public void rotateOneSection()
    {
        if (!isCarouselOn)
        {
            isCarouselOn = true;
        }

        carousel.setPower(0.3);

        if (!magneticSwitch.getState())
        {
            carousel.setPower(0);
            isCarouselOn = false;
        }
    }

    //------------------------FINISH DECLARING METHODS------------------------//
}