package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    int goalPiplineRed = 5;
    int goalPiplineBlue = 6;

    int motifPipline = 7;

    final double shooterP = 40.132;
    final double shooterI = 0;
    final double shooterD = 0;
    final double shooterF = 13.727;

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

        telemetry.setMsTransmissionInterval(11);




        waitForStart();

        if(opModeIsActive())
        {
            turnToGoal(goalPiplineBlue);

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
}
