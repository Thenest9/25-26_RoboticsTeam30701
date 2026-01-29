package org.firstinspires.ftc.teamcode.pedroPathing;

//Import to be able to use opMode, so you can sleep the robot and check if it is active
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Import to be able to use the carousel, so you can get the balls to the flywheels

//import com.pedropathing.util.Timer;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;

//Import to be able to use the wheels motors, so you can move the robot
import com.qualcomm.robotcore.hardware.DcMotor;

//Import to be able to move the fly wheels, so you can shoot the balls
import com.qualcomm.robotcore.hardware.DcMotorEx;

//Import to be able to use telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry;

//Imports to be able to use Lists and ArrayLists; Used in getMotif()

//Imports to use the Limelight; Used in getMotif() and positionToGoal
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

//Import to use the gate servo, used for indexing
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

//Import to use the color sensor, used for indexing and checking color for the ball
import com.qualcomm.robotcore.hardware.ColorSensor;


public class LibraryPedro
{
    //Making the DcMotors for the tires, the ramp, and the intake
    private DcMotor intake, ramp;


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

    DigitalChannel touchSensorBot;
    DigitalChannel touchSensorTop;
    public boolean isShooting;
    public boolean isIntaking;
    private Timer intakeTimer;
    private Timer shootTimer;
    private Timer rampTimer;

    public LibraryPedro(DcMotorEx oR, DcMotorEx oL, CRServo c, Telemetry t, Limelight3A ll, DcMotor i, DcMotor r, Servo g, ColorSensor cs, DigitalChannel top, DigitalChannel bot)
    {
        outputRight = oR;
        outputLeft = oL;
        carousel = c;
        telemetry = t;
        limelight = ll;
        intake = i;
        ramp = r;
        gate = g;
        colorSensor = cs;
        touchSensorTop = top;
        touchSensorBot = bot;

        shootTimer = new Timer();
        intakeTimer = new Timer();
        rampTimer = new Timer();
    }

    //Being able to make the Library object without having all the constructors at the cost of
    //passing through the reference objects for every call
    public LibraryPedro()
    {

    }
    public double getShootTimer()
    {
        return shootTimer.getElapsedTimeSeconds();
    }
    public double getIntakeTimer()
    {
        return intakeTimer.getElapsedTimeSeconds();
    }
    public double getRampTimer()
    {
        return rampTimer.getElapsedTimeSeconds();
    }
    public void initTimer()
    {
        if(shootTimer==null)
        {
            shootTimer = new Timer();
        }
        if(intakeTimer==null) {
            intakeTimer = new Timer();
        }
        if(rampTimer==null)
        {
            rampTimer= new Timer();
        }
    }
//    public void intakeProcess1()
//    {
//        rampDown();
//        IntakeStart();
//    }
    public void IntakeStart()
    {
        intake.setPower(1);
        carousel.setPower(1);

        intakeTimer.resetTimer();//TODO: create individual timers for shoot and intake
        isIntaking = true;
    }
    public void finishIntake()
    {
        if(isIntaking)
        {
            if(intakeTimer.getElapsedTimeSeconds()>5.0)
            {
                intake.setPower(0);
                carousel.setPower(0.0);
                isIntaking=false;
            }
        }
    }
    public void rampDown() {
        while(touchSensorBot.getState())
        {
            ramp.setPower(-0.5);
            intake.setPower(-1);
        }
        ramp.setPower(0.0);
        intake.setPower(0.0);

    }
    public void rampUp()
    {
        while(touchSensorTop.getState())
        {
            ramp.setPower(-0.5);
            intake.setPower(1);
        }
            ramp.setPower(0);
            intake.setPower(0);
    }
    public void shootThree(int velocity)
    {
        outputRight.setVelocity(velocity);
        outputLeft.setVelocity(velocity);
        carousel.setPower(-1);


        shootTimer.resetTimer();
        isShooting=true;
    }
    public void updateShoot()
    {
        if (isShooting)
        {
            if (shootTimer.getElapsedTimeSeconds()>3.0)
            {
                carousel.setPower(0);
                outputLeft.setVelocity(0);
                outputRight.setVelocity(0);
                isShooting = false;
            }
        }
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
//                telemetry.addData("Color:", "Purple, Purple, Green");
            }


            else if(detectedTagId == 22)
            {
                motif = "pgp";
//                telemetry.addData("Color:", "Purple, Green, Purple");
            }


            else if(detectedTagId == 21)
            {
                motif = "gpp";
//                telemetry.addData("Color:", "Green, Purple, Purple");
            }
        }
        return motif;
    }

//    public void orderBalls(String motif, String order)
//    {
//        //Moves the gate to the front area of the carousel, the values are absolute
//        gate.setPosition(0.67);
//
//        telemetry.addData("Motif: ", motif);
//        telemetry.addData("Order: ", order);
//        telemetry.update();
//
//        if(orderPossible(order))
//        {
//            while(!motif.equals(order))
//            {
//                //Spins the carousel to the next section
//                carousel.setPower(0.25);
//                opMode.sleep(1250);
//
//                //Stops the carousel from spinning
//                carousel.setPower(0);
//                opMode.sleep(1000);
//
//                //Move the last item in the string to the front
//                order = order.substring(2) + order.substring(0,2);
//            }
//        }
//        telemetry.addData("Motif: ", motif);
//        telemetry.addData("Order: ", order);
//        telemetry.addData("Is It Possible: ", orderPossible(order));
//        telemetry.update();
//
//        gate.setPosition(0.95);
//    }
//
//    private boolean orderPossible(String order)
//    {
//        if(order.contains("g") && order.contains("p") && order.substring(order.indexOf("p") + 1).contains("p"))
//        {
//            return true;
//        }
//        //if it doesn't have 2 purples and one green, it returns false
//        return false;
//    }
////
//    //Spins through the carousel and gets the values of the order of the balls
//    public String getOrder()
//    {
//        String order = "";
//        for(int i = 0; i < 3; i++)
//        {
//            //Spins the carousel to the next section
//            carousel.setPower(0.25);
//            opMode.sleep(1000);
//
//            //Spins the carousel up a little bit
//            carousel.setPower(-0.2);
//            opMode.sleep(220);
//
//            //Stops the carousel from spinning
//            carousel.setPower(0);
//            opMode.sleep(280);
//
//            //Adds the current ball to the string already made
//            order = checkRGB() + order;
//
//            //Telemetry for us knowing the details of each ball
//            telemetry.addData("CS Red: ", colorSensor.red());
//            telemetry.addData("CS Green: ", colorSensor.green());
//            telemetry.addData("CS Blue: ", colorSensor.blue());
//            telemetry.addData("Current Order: ", order);
//            telemetry.update();
//        }
//
//        //Returns the string filled with the correct order
//        return order;
    }
//
//    //Checks what type of ball is there and returns a string depending on what is there
    //g for a green ball
    //p for a purple ball
    //n for no ball
    //If the color sensor is pointing at the hole, it wont detect the balls color and would print n
//    private String checkRGB() {
//        int red = colorSensor.red();
//        int green = colorSensor.green();
//        int blue = colorSensor.blue();
//
//        //This else if statements checks if there is a purple ball there
//        //The red value for a purple ball is 2440 - 2445
//        //The green value for a purple ball is 2945 - 2960
//        //The blue value for a purple ball is 4945 - 4950
//        //Leaving a buffer of about 1000
//        if (blue >= 3000 && red >= 1750)
//        {
//            return "p";
//        }
//        //Prints out there is a green ball
//        //The red value for a green ball is 690 - 695
//        //The green value for a green ball is 2685 - 2690
//        //The blue value for a green ball is 1985 - 1990
//        //Leaving a buffer of about 100
//        else if(blue >= 1000)
//        {
//            return "g";
//        }
//
//        //This if statements checks if there is no ball there
//        //The red value for nothing there is 93 - 95
//        //The green value for nothing there is 151 - 154
//        //The blue value for nothing there is 137 - 139
//        return "n";
//    }