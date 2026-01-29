package org.firstinspires.ftc.teamcode;

//Import to be able to use the Autonomous section

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ValuesTesting")
public class ValuesTesting extends LinearOpMode
{
    Limelight3A limelight;

    DcMotor FrontLeft,FrontRight,RearLeft,RearRight, intake, ramp;
    DcMotorEx outputRight, outputLeft;

    ColorSensor colorSensor;

    CRServo carousel;
    Servo gate;

    final double shooterP = 40.132;
    final double shooterI = 0;
    final double shooterD = 0;
    final double shooterF = 13.727;

    String motif;
    String order;

    @Override
    public void runOpMode()
    {

        // Initialize drive motors
        FrontLeft = hardwareMap.get(DcMotor.class,"FrontLeft");// MOTOR 0
        FrontRight = hardwareMap.get(DcMotor.class,"FrontRight");// MOTOR 3
        RearLeft = hardwareMap.get(DcMotor.class,"RearLeft");// MOTOR 1
        RearRight = hardwareMap.get(DcMotor.class,"RearRight");// MOTOR 2

        outputRight = hardwareMap.get(DcMotorEx.class, "RightOutput");
        outputRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outputRight.setDirection(DcMotorSimple.Direction.REVERSE);

        outputLeft = hardwareMap.get(DcMotorEx.class, "LeftOutput");
        outputLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outputRight.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        outputLeft.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);

        intake = hardwareMap.get(DcMotor.class, "Intake");

        carousel = hardwareMap.get(CRServo.class, "Carousel");

        ramp = hardwareMap.get(DcMotor.class, "rampIntakeOuttake");
        gate = hardwareMap.get(Servo.class, "gate");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();

        Library lib = new Library(FrontLeft, FrontRight, RearLeft, RearRight, this, outputRight, outputLeft, carousel, telemetry, limelight, intake, ramp, gate, colorSensor);

        waitForStart();

        if(opModeIsActive())
        {
            //Moves the gate to the front area of the carousel, the values are absolute
            gate.setPosition(0.67);

            //Gets the values of where the balls are
            motif = lib.getMotif();
            order = getOrder();

            telemetry.addData("Motif: ", motif);
            telemetry.addData("Order: ", order);
            telemetry.update();

            if(possible(order))
            {
                while(!motif.equals(order))
                {
                    //Spins the carousel to the next section
                    carousel.setPower(0.25);
                    sleep(1000);

                    //Stops the carousel from spinning
                    carousel.setPower(0);
                    sleep(1000);

                    //Move the last item in the string to the front
                    order = order.substring(2) + order.substring(0,2);
                }
            }
            telemetry.addData("Motif: ", motif);
            telemetry.addData("Order: ", order);
            telemetry.addData("Is It Possible: ", possible(order));
            telemetry.update();

            carousel.setPower(0.25);
            sleep(250);
            carousel.setPower(0);

            gate.setPosition(0.95);

            lib.shootThree(1267);

            sleep(5000);
        }
    }

    //Uses the motif value and the order to return a boolean value which tells you if it is
    //possible or not to match the motif
    public boolean possible(String order)
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
            sleep(1000);

            //Spins the carousel up a little bit
            carousel.setPower(-0.2);
            sleep(220);

            //Stops the carousel from spinning
            carousel.setPower(0);
            sleep(280);

            //Adds the current ball to the string already made
            order = checkRGB() + order;

            //Telemetry for us knowing the details of each ball
            telemetry.addData("CS Red: ", colorSensor.red());
            telemetry.addData("CS Green: ", colorSensor.green());
            telemetry.addData("CS Blue: ", colorSensor.blue());
            telemetry.addData("Current Order: ", order);
            telemetry.update();

            //Time so we can read the data
            sleep(2000);
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
}