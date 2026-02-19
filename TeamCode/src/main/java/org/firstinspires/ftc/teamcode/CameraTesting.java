package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Camera Testing")
public class CameraTesting extends LinearOpMode
{
    Limelight3A limelight;
    Servo gate;
    DcMotor FrontLeft, FrontRight, RearLeft, RearRight, intake;
    DcMotorEx outputRight, outputLeft;
    DcMotor ramp;
    CRServo carousel;
    ColorSensor colorSensor;
    DigitalChannel magneticLimitSwitch;


    int goalPiplineRed = 5;
    int goalPiplineBlue = 6;

    int motifPipline = 7;

    final double shooterP = 40.132;
    final double shooterI = 0;
    final double shooterD = 0;
    final double shooterF = 13.727;

    String motif = "";


    @Override
    public void runOpMode()
    {

        // Initialize drive motors
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");// MOTOR 0
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");// MOTOR 3
        RearLeft = hardwareMap.get(DcMotor.class, "RearLeft");// MOTOR 1
        RearRight = hardwareMap.get(DcMotor.class, "RearRight");// MOTOR 2

        outputRight = hardwareMap.get(DcMotorEx.class, "RightOutput");
        outputLeft = hardwareMap.get(DcMotorEx.class, "LeftOutput");

        ramp = hardwareMap.get(DcMotor.class, "rampIntakeOuttake");
        gate = hardwareMap.get(Servo.class, "gate");

        carousel = hardwareMap.get(CRServo.class, "Carousel");

        outputRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outputLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outputRight.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        outputLeft.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        outputRight.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        intake = hardwareMap.get(DcMotor.class, "Intake");


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(motifPipline);
        limelight.start();

        magneticLimitSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch");
        magneticLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        telemetry.setMsTransmissionInterval(11);




        waitForStart();

        if(opModeIsActive())
        {
            motif = getMotif();
            if(!motif.isEmpty())
            {
                orderBalls(motif, "ppg");
            }
            telemetry.update();
        }
    }

    public void setMotorsPower(double fLSpeed, double fRSpeed, double rLSpeed, double rRSpeed)//function to set all motors to the same speed
    {
        //set the powers to drive the robot
        FrontLeft.setPower(fLSpeed);
        FrontRight.setPower(fRSpeed);
        RearLeft.setPower(rLSpeed);
        RearRight.setPower(rRSpeed);
    }



    public void turnToGoal(int pipeline)
    {
        // Switch Limelight to the correct pipeline
        limelight.pipelineSwitch(pipeline);

        // Wait a short moment for the camera to update
        sleep(100);

        boolean aligned = false;
        boolean targetVisible = false;
        while(opModeIsActive() && !aligned)
        {
            LLResult result = limelight.getLatestResult();
            if(result.isValid())
            {
                targetVisible = true;
                double tx = result.getTx(); // Horizontal offset from target
                double turnPower = tx * 0.025; // Scale factor for turning

                // Clamp the power to avoid being too slow or too fast
                double maxPower = 0.4;
                double minPowerTurn = 0.2;
                double maxPowerTurn = 0.35;
                double degreeFromCenter = 6.7;

                if (turnPower > 0 && turnPower < minPowerTurn)
                {
                    turnPower = minPowerTurn;
                }
                else if (turnPower < 0 && turnPower < -minPowerTurn)
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
                if (Math.abs(tx) < degreeFromCenter + 1)
                {
                    aligned = true;

                }

            }
            else
            {
                targetVisible = false;
                setMotorsPower(-0.5, -0.5, -0.5, -0.5);
            }
            telemetry.addData("Turning to goal", !aligned ? "In progress" : "Aligned");
            telemetry.addData("Target visible", targetVisible);
            telemetry.update();
        }

        // Stop all motors when aligned
        setMotorsPower(0, 0, 0, 0);
    }
    public void orderBalls(String motif, String order)
    {
//        isDoneIntaking=true;
        //Moves the gate to the front area of the carousel, the values are absolute
        gate.setPosition(0.67);

        telemetry.addData("Motif: ", motif);
        telemetry.addData("Order: ", order);
        telemetry.update();
        while(!motif.equals(order))
        {
            //isDoneSpindexing=false;
            //Spins the carousel to the next section
            while(!magneticLimitSwitch.getState())
            {
                carousel.setPower(0.2);
            }
            carousel.setPower(0);
            while(magneticLimitSwitch.getState())
            {
                carousel.setPower(0.3);
            }
            //Stops the carousel from spinning
            carousel.setPower(0);
            //Move the last item in the string to the front
            order = order.substring(2) + order.substring(0, 2);
        }
        telemetry.addData("Motif: ", motif);
        telemetry.addData("Order: ", order);
        telemetry.update();

        gate.setPosition(0.95);
        //isDoneSpindexing=true;
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
}
