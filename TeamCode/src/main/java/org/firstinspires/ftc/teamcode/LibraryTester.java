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

@Autonomous(name = "LibraryTester")
public class LibraryTester extends LinearOpMode
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
    String order = "ppg";

    @Override
    public void runOpMode() {

        // Initialize drive motors
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");// MOTOR 0
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");// MOTOR 3
        RearLeft = hardwareMap.get(DcMotor.class, "RearLeft");// MOTOR 1
        RearRight = hardwareMap.get(DcMotor.class, "RearRight");// MOTOR 2

        outputRight = hardwareMap.get(DcMotorEx.class, "RightOutput");
        outputRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outputRight.setDirection(DcMotorSimple.Direction.REVERSE);

        outputLeft = hardwareMap.get(DcMotorEx.class, "LeftOutput");
        outputLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outputRight.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        outputLeft.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);

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

        if (opModeIsActive())
        {
            motif = lib.getMotif();
            lib.orderBalls(motif, order);
            lib.shootThree(1267);
        }
    }
}