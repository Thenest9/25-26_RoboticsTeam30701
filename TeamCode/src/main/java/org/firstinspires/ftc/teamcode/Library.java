package org.firstinspires.ftc.teamcode;

//Import to be able to use opMode, so you can sleep the robot and check if it is active
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Import to be able to use the carousel, so you can get the balls to the flywheels
import com.qualcomm.robotcore.hardware.CRServo;

//Import to be able to use the wheels motors, so you can move the robot
import com.qualcomm.robotcore.hardware.DcMotor;

//Import to be able to move the fly wheels, so you can shoot the balls
import com.qualcomm.robotcore.hardware.DcMotorEx;

//Import to be able to use telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry;

//Imports to use the Limelight; Used in getMotif() and positionToGoal
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;

//Import to use the gate servo, used for indexing
import com.qualcomm.robotcore.hardware.Servo;

//Import to use the color sensor, used for indexing and checking color for the ball
import com.qualcomm.robotcore.hardware.ColorSensor;

public class Library
{
    //Making the DcMotors for the tires, the ramp, and the intake
    private DcMotor FrontLeft, FrontRight, RearLeft, RearRight, intake, ramp;

    //Passing through this in the constructor to be able to use "sleep()"
    private LinearOpMode opMode;

    //Making the DcMotorEx for the fly wheels
    private DcMotorEx outputRight, outputLeft;

    //Making the CRServo for the carousel
    private CRServo carousel;

    //Passing through this in the constructor to be able to use "telemetry"
    private Telemetry telemetry;

    //Making the limelight to be able to detect the april tags
    private Limelight3A limelight;

    //Making the gate to have the gate open or closed depending on intake or output
    private Servo gate;

    //Making the colorSensor to be able to check the color od the balls
    private ColorSensor colorSensor;

    /*Making the DcMotors defined in the main class have the same reference points as the ones made here
    Takes in 4 DcMotor Objects and they have to be in order from left to right, up to down
    Takes in the linearOpMode but in teh constructor, just pass through "this"
    Takes in 2 DcMotorEx for the fly wheels, Right then Left
    Takes in the CRServo motor for the carousel
    Takes in the telemetry to be able to use telemetry
    Takes in the limelight to be able to detect april tags
    Takes in the intake to turn it on to take balls into the carousel
    Takes in the ramp to set it to intake to output and vice versa
    Takes in the gate to set the indexing to output or input
    Takes in the color sensor to see which balls are what colors
    */
    public Library(DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, LinearOpMode mode, DcMotorEx oR, DcMotorEx oL, CRServo c, Telemetry t, Limelight3A ll, DcMotor i, DcMotor r, Servo g, ColorSensor cs)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        opMode = mode;
        outputRight = oR;
        outputLeft = oL;
        carousel = c;
        telemetry = t;
        limelight = ll;
        intake = i;
        ramp = r;
        gate = g;
        colorSensor = cs;
    }

    //Being able to make the Library object without having all the constructors at the cost of
    //passing through the reference objects for every call
    public Library()
    {
    }

    /*Sets the Motors Power speed
    Takes in double parameters, they decide the speed at which the wheels will move
    They have to be in the order: left to right, up to down
    */
    private void setMotorsPower(double fLSpeed,double fRSpeed,double rLSpeed,double rRSpeed)
    {
        telemetry.update();
        FrontLeft.setPower(fLSpeed);
        FrontRight.setPower(fRSpeed);
        RearLeft.setPower(rLSpeed);
        RearRight.setPower(rRSpeed);
    }

    /*Sets the motors for a certain speed for a time given and then turns them back off
    The power of the 4 motors have to passed through as well as the time
    This can only be used inside the class; it is a helper method for the moving methods
    */
    private void runMotorsFor(double fl, double fr, double rl, double rr, long duration)
    {
        long start = System.currentTimeMillis();
        while(opMode.opModeIsActive() && System.currentTimeMillis() - start < duration)
        {
            setMotorsPower(fl, fr, rl, rr);
            opMode.sleep(5);
        }
        telemetry.addData("Movement: ", "Stop");
        setMotorsPower(0,0,0,0);
    }

    /*Makes the robot go Backwards
    Takes in a double parameter telling the robot at which speed it will move the wheels
    Takes in a long parameter telling the robot for how long to move the wheels
    */
    public void driveBackward (double power, long timeMillis)
    {
        telemetry.addData("Movement: ", "Backward");
        runMotorsFor(power, -power, power, -power, timeMillis);
    }

    /*Makes the robot move backwards
    the double power passes through the speed at which to move backward
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveBackward (double power)
    {
        telemetry.addData("Movement: ", "Backward");
        setMotorsPower(power, -power, power, -power);
    }

    /*Makes the robot move backwards (when not using the "long" constructor)
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    A Long to move the wheels for that amount of time
     */
    public void driveBackward (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power, long timeMillis)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        runMotorsFor(power, -power, power, -power, timeMillis);
    }

    /*Makes the robot move backwards (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveBackward (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        setMotorsPower(power, -power, power, -power);
    }

    /*Makes the robot go Forwards
    Takes in a double parameter telling the robot at which speed it will move the wheels
    Takes in a long parameter telling the robot for how long to move the wheels
    */
    public void driveForward ( double power, long timeMillis)
    {
        telemetry.addData("Movement: ", "Forward");
        runMotorsFor(-power, power, -power, power, timeMillis);
    }

    /*Makes the robot go Forwards
    the double power passes through the speed at which to move forward
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveForward (double power)
    {
        telemetry.addData("Movement: ", "Forward");
        setMotorsPower(-power, power, -power, power);
    }

    /*Makes the robot go Forwards (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    A Long to move the wheels for that amount of time
     */
    public void driveForward (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power, long timeMillis)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        runMotorsFor(-power, power, -power, power, timeMillis);
    }

    /*Makes the robot go Forwards (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveForward (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        setMotorsPower(-power, power, -power, power);
    }

    /*Makes the robot go Right
    Takes in a double parameter telling the robot at which speed it will move the wheels
    Takes in a long parameter telling the robot for how long to move the wheels
    */
    public void driveRight ( double power, long timeMillis)
    {
        telemetry.addData("Movement: ", "Right");
        runMotorsFor(-power, -power, power, power, timeMillis);
    }

    /*Makes the robot move Right
    the double power passes through the speed at which to move right
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveRight (double power)
    {
        telemetry.addData("Movement: ", "Right");
        setMotorsPower(-power, -power, power, power);
    }

    /*Makes the robot go Right (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    A Long to move the wheels for that amount of time
     */
    public void driveRight (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power, long timeMillis)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        runMotorsFor(-power, -power, power, power, timeMillis);
    }

    /*Makes the robot go Right (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveRight (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        setMotorsPower(-power, -power, power, power);
    }

    /*Makes the robot go Left
    Takes in a double parameter telling the robot at which speed it will move the wheels
    Takes in a long parameter telling the robot for how long to move the wheels
    */
    public void driveLeft ( double power, long timeMillis)
    {
        telemetry.addData("Movement: ", "Left");
        runMotorsFor(power, power, -power, -power, timeMillis);
    }

    /*Makes the robot move Left
    the double power passes through the speed at which to move left
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveLeft (double power)
    {
        telemetry.addData("Movement: ", "Left");
        setMotorsPower(power, power, -power, -power);
    }

    /*Makes the robot go Left (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    A Long to move the wheels for that amount of time
     */
    public void driveLeft (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power, long timeMillis)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        runMotorsFor(power, power, -power, -power, timeMillis);
    }

    /*Makes the robot go Left (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveLeft (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        setMotorsPower(power, power, -power, -power);
    }

    /*Makes the robot go Top-Left
    Takes in a double parameter telling the robot at which speed it will move the wheels
    Takes in a long parameter telling the robot for how long to move the wheels
    */
    public void driveUpLeft ( double power, long timeMillis)
    {
        telemetry.addData("Movement: ", "Up-Left");
        runMotorsFor(0, power, -power, 0, timeMillis);
    }

    /*Makes the robot move Top-Left
    the double power passes through the speed at which to move top-left
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveUpLeft (double power)
    {
        telemetry.addData("Movement: ", "Up-Left");
        setMotorsPower(0, power, -power, 0);
    }

    /*Makes the robot go Top-Left (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    A Long to move the wheels for that amount of time
     */
    public void driveUpLeft (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power, long timeMillis)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        runMotorsFor(0, power, -power, 0, timeMillis);
    }

    /*Makes the robot go Top-Left (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveUpLeft (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        setMotorsPower(0, power, -power, 0);
    }

    /*Makes the robot go Top-Right
    Takes in a double parameter telling the robot at which speed it will move the wheels
    Takes in a long parameter telling the robot for how long to move the wheels
    */
    public void driveUpRight ( double power, long timeMillis)
    {
        telemetry.addData("Movement: ", "Up-Right");
        runMotorsFor(-power, 0, 0, power, timeMillis);
    }

    /*Makes the robot move Top-Right
    the double power passes through the speed at which to move top-right
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveUpRight (double power)
    {
        telemetry.addData("Movement: ", "Up-Right");
        setMotorsPower(-power, 0, 0, power);
    }

    /*Makes the robot go Top-Right (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    A Long to move the wheels for that amount of time
     */
    public void driveUpRight (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power, long timeMillis)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        runMotorsFor(-power, 0, 0, power, timeMillis);
    }

    /*Makes the robot go Top-Right (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveUpRight (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        setMotorsPower(-power, 0, 0, power);
    }

    /*Makes the robot go Down-Right
    Takes in a double parameter telling the robot at which speed it will move the wheels
    Takes in a long parameter telling the robot for how long to move the wheels
    */
    public void driveDownRight ( double power, long timeMillis)
    {
        telemetry.addData("Movement: ", "Down-Right");
        runMotorsFor(0, -power, power, 0, timeMillis);
    }

    /*Makes the robot move Down-Right
    the double power passes through the speed at which to move down-right
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveDownRight (double power)
    {
        telemetry.addData("Movement: ", "Down-Right");
        setMotorsPower(0, -power, power, 0);
    }

    /*Makes the robot go Down-Right (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    A Long to move the wheels for that amount of time
     */
    public void driveDownRight (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power, long timeMillis)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        runMotorsFor(0, -power, power, 0, timeMillis);
    }

    /*Makes the robot go Down-Right (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveDownRight (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        setMotorsPower(0, -power, power, 0);
    }

    /*Makes the robot go Down-Left
    Takes in a double parameter telling the robot at which speed it will move the wheels
    Takes in a long parameter telling the robot for how long to move the wheels
    */
    public void driveDownLeft ( double power, long timeMillis)
    {
        telemetry.addData("Movement: ", "Down-Left");
        runMotorsFor(power, 0, 0, -power, timeMillis);
    }

    /*Makes the robot move Down-Left
    the double power passes through the speed at which to move Down-Left
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveDownLeft (double power)
    {
        telemetry.addData("Movement: ", "Down-Left");
        setMotorsPower(power, 0, 0, -power);
    }

    /*Makes the robot go Down-Left (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    A Long to move the wheels for that amount of time
     */
    public void driveDownLeft (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power, long timeMillis)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        runMotorsFor(power, 0, 0, -power, timeMillis);
    }

    /*Makes the robot go Down-Left (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void driveDownLeft (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        setMotorsPower(power, 0, 0, -power);
    }

    /*Makes the robot turn Right
    Takes in a double parameter telling the robot at which speed it will move the wheels
    Takes in a long parameter telling the robot for how long to move the wheels
    */
    public void turnRight(double power, long timeMillis)
    {
        telemetry.addData("Movement: ", "Turn Right");
        runMotorsFor(-power, -power,-power, -power, timeMillis);
    }

    /*Makes the robot turn Right
    the double power passes through the speed at which to turn right
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void turnRight (double power)
    {
        telemetry.addData("Movement: ", "Turn Right");
        setMotorsPower(-power, -power, -power, -power);
    }

    /*Makes the robot turn Right (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    A Long to move the wheels for that amount of time
     */
    public void turnRight (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power, long timeMillis)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        runMotorsFor(-power, -power, -power, -power, timeMillis);
    }

    /*Makes the robot turn Right (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void turnRight (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        setMotorsPower(-power, -power, -power, -power);
    }

    /*Makes the robot turn Left
    Takes in a double parameter telling the robot at which speed it will move the wheels
    Takes in a long parameter telling the robot for how long to move the wheels
    */
    public void turnLeft(double power, long timeMillis)
    {
        telemetry.addData("Movement: ", "Turn Left");
        runMotorsFor(power, power, power, power, timeMillis);
    }

    /*Makes the robot turn Left
    the double power passes through the speed at which to turn left
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void turnLeft (double power)
    {
        telemetry.addData("Movement: ", "Turn Left");
        setMotorsPower(power, power, power, power);
    }

    /*Makes the robot turn Left (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    A Long to move the wheels for that amount of time
     */
    public void turnLeft (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power, long timeMillis)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        runMotorsFor(power, power, power, power, timeMillis);
    }

    /*Makes the robot turn Left (when not using the "long constructor")
    4 DcMotors that pass through the reference to the wheels going from order left to right, up and down
    A double to move the wheels at that speed
    this doesn't stop the robot at all, meaning your gonna have to set the powers of the wheels to
    0 manually
     */
    public void turnLeft (DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, double power)
    {
        FrontLeft = fl;
        FrontRight = fr;
        RearLeft = rl;
        RearRight = rr;
        telemetry.addData("Movement: ", "Backward");
        setMotorsPower(power, power, power, power);
    }

    //Stops the robot
    public void stopMoving()
    {
        setMotorsPower(0, 0, 0, 0);
    }

    /*Shoots the ball out of the carousel
    Takes in an int parameter telling the robot to spin the fly wheels at what speed
    Takes in an long parameter telling the robot how long to charge for before shooting
    */
    public void shoot(double velocity)
    {
        outputRight.setVelocity(velocity);
        outputLeft.setVelocity(velocity);

        carousel.setPower(-0.25);
        opMode.sleep(1000);

        carousel.setPower(0.0);
        outputRight.setVelocity(0);
        outputLeft.setVelocity(0);

        telemetry.addData("Shooter", "Stopped");
        telemetry.update();
    }

    public void shootThree(double velocity)
    {
        outputRight.setVelocity(velocity);
        outputLeft.setVelocity(velocity);
        carousel.setPower(-0.67);
        opMode.sleep(3000);

        carousel.setPower(0);
        outputLeft.setVelocity(0);
        outputRight.setVelocity(0);
    }

    public String getMotif()
    {
        String motif = "";
        limelight.pipelineSwitch(7);
        LLResult result = limelight.getLatestResult();

        //Pipeline that is prepared to check for the tags of the motifs


        if (result.isValid() && !result.getFiducialResults().isEmpty())
        {
            //Gets the ID number of the tag; the number is an int; either 21, 22, or 23
            LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
            int detectedTagId = fiducial.getFiducialId();
            telemetry.addData("AprilTag ID", detectedTagId);

            if (detectedTagId == 23)
            {
                motif = "ppg";
                telemetry.addData("Color:", "Purple, Purple, Green");
            }


            else if(detectedTagId == 22)
            {
                motif = "pgp";
                telemetry.addData("Color:", "Purple, Green, Purple");
            }


            else if(detectedTagId == 21)
            {
                motif = "gpp";
                telemetry.addData("Color:", "Green, Purple, Purple");
            }
        }

        telemetry.update();
        return motif;
    }

    public void orderBalls(String motif, String order)
    {
        //Moves the gate to the front area of the carousel, the values are absolute
        gate.setPosition(0.67);

        telemetry.addData("Motif: ", motif);
        telemetry.addData("Order: ", order);
        telemetry.update();

        if(orderPossible(order))
        {
            carousel.setPower(0);
            opMode.sleep(500);
            while(!motif.equals(order))
            {
                //Spins the carousel to the next section
                carousel.setPower(0.25);
                opMode.sleep(1000);

                //Stops the carousel from spinning
                carousel.setPower(0);
                opMode.sleep(1000);

                //Move the last item in the string to the front
                order = order.substring(2) + order.substring(0,2);
            }
        }
        telemetry.addData("Motif: ", motif);
        telemetry.addData("Order: ", order);
        telemetry.addData("Is It Possible: ", orderPossible(order));
        telemetry.update();

        gate.setPosition(0.95);
    }

    private boolean orderPossible(String order)
    {
        if(order.contains("g") && order.contains("p") && order.substring(order.indexOf("p") + 1).contains("p"))
        {
            return true;
        }
        //if it doesn't have 2 purples and one green, it returns false
        return false;
    }

    //Spins through the carousel and gets the values of the order of the balls
    public String getOrder()
    {
        String order = "";

        for(int i = 0; i < 3; i++)
        {
            //Spins the carousel to the next section
            carousel.setPower(0.25);
            opMode.sleep(1000);

            //Spins the carousel up a little bit
            carousel.setPower(-0.2);
            opMode.sleep(220);

            //Stops the carousel from spinning
            carousel.setPower(0);
            opMode.sleep(280);

            //Adds the current ball to the string already made
            order = checkRGB() + order;

            //Telemetry for us knowing the details of each ball
            telemetry.addData("CS Red: ", colorSensor.red());
            telemetry.addData("CS Green: ", colorSensor.green());
            telemetry.addData("CS Blue: ", colorSensor.blue());
            telemetry.addData("Current Order: ", order);
            telemetry.update();
        }

        //Returns the string filled with the correct order
        return order;
    }

    //Checks what type of ball is there and returns a string depending on what is there
    //g for a green ball
    //p for a purple ball
    //n for no ball
    //If the color sensor is pointing at the hole, it wont detect the balls color and would print n
    private String checkRGB()
    {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        //This else if statements checks if there is a purple ball there
        //The red value for a purple ball is 2440 - 2445
        //The green value for a purple ball is 2945 - 2960
        //The blue value for a purple ball is 4945 - 4950
        //Leaving a buffer of about 1000
        if (blue >= 3000 && red >= 1750)
        {
            return "p";
        }
        //Prints out there is a green ball
        //The red value for a green ball is 690 - 695
        //The green value for a green ball is 2685 - 2690
        //The blue value for a green ball is 1985 - 1990
        //Leaving a buffer of about 100
        else if(blue >= 1000)
        {
            return "g";
        }

        //This if statements checks if there is no ball there
        //The red value for nothing there is 93 - 95
        //The green value for nothing there is 151 - 154
        //The blue value for nothing there is 137 - 139
        return "n";
    }

    public void turnToAprilTag(int pipeline)
    {
        limelight.pipelineSwitch(pipeline);
        opMode.sleep(100);

        boolean aligned = false;
        boolean targetVisible = false;
        while(opMode.opModeIsActive() && !aligned)
        {
            LLResult result = limelight.getLatestResult();
            if(result.isValid())
            {
                targetVisible = true;
                double tx = result.getTx(); // Horizontal offset from target
                double turnPower = tx * 0.02; // Scale factor for turning

                // Clamp the power to avoid being too slow or too fast
                double minPowerTurn = 0.2;
                double maxPowerTurn = 0.35;
                double degreeFromCenter = 2;

                if (turnPower > 0 && turnPower < minPowerTurn)
                {
                    turnPower = minPowerTurn;
                }
                else if (turnPower < 0 && turnPower > -minPowerTurn)
                {
                    turnPower = -minPowerTurn;
                }
                if (turnPower > maxPowerTurn)
                {
                    turnPower = maxPowerTurn;
                }
                else if (turnPower < -maxPowerTurn)
                {
                    turnPower = -maxPowerTurn;
                }

                // Set drivetrain to turn
                setMotorsPower(-turnPower, -turnPower, -turnPower, -turnPower);

                // Check if we’re close enough to the target
                if (tx < 10 && tx > -1.5)
                {
                    aligned = true;

                }

                telemetry.addData("Horizontal offset (Absolute value)", Math.abs(tx));

            }
            else
            {
                targetVisible = false;
                setMotorsPower(-0.25, -0.25, -0.25, -0.25);
            }
            telemetry.addData("Turning to goal", !aligned ? "In progress" : "Aligned");
            telemetry.addData("Target visible", targetVisible);
            telemetry.update();
        }

        setMotorsPower(0, 0, 0, 0);
    }

    //Runs two tasks that are passed through as parameters
    public void runTogether(Runnable run1, Runnable run2)
    {
        //2 thread objects that run the 2 tasks
        Thread turnGoal = new Thread(run1);
        Thread orderBalls = new Thread(run2);

        //Starts both of the threads
        turnGoal.start();
        orderBalls.start();

        //Joins the 2 threads back to the main thread
        try
        {
            turnGoal.join();
            orderBalls.join();
        }
        //If they have an error, it passes a error in the log
        catch (InterruptedException e)
        {
            telemetry.addData("Joining Threads: ", "Failed");
            telemetry.update();
        }
    }

    //Method is in inches
    public double getAprilTagDistance(double ty)
    {
        double cameraHeight = 15.5; //Height of camera from floor to center
        double tagHeight = 29.5; //Height of tag from floor to center
        double cameraAmountAngle = 6.504; //The angle the limelight is mounted at

        //The angle at which the camera is looking up towards the apriltag
        double angleRad = Math.toRadians(cameraAmountAngle + ty);

        //tagHeight - cameraHeight is the length of the side from the camera height to
        // the april tag height.
        double height = (tagHeight - cameraHeight);

        //Math.tan(angleRad) returns the steepness of the hypotenuse.
        double steepness = Math.tan(angleRad);

        //We divide the height from the camera to the april tag, and divide it by the steepness
        //which cancels out the height to give you the distance.
        return height / steepness;
    }

}