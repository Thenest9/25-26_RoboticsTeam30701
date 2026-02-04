package org.firstinspires.ftc.teamcode;

//Import to be able to use the Autonomous section
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Import to be able to use parts of LinearOpMode here
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Import to be able to make the limelight and hardware map it
import com.qualcomm.hardware.limelightvision.Limelight3A;

//Import to be able to make the wheels, intake, and ramp and be able to hardware map it
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Import to be able to make the fly wheels and be able to hardware map it
import com.qualcomm.robotcore.hardware.DcMotorEx;

//Import to be able to make the color sensor and be able to hardware map it
import com.qualcomm.robotcore.hardware.ColorSensor;

//Import to be able to make the carousel and be able to hardware map it
import com.qualcomm.robotcore.hardware.CRServo;

//Import to be able to make the gate and be able to hardware map it
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
    double shooterP = 48.72995;
    double shooterI = 0;
    double shooterD = 0;
    double shooterF = 13.13319;

    double velocity = 1400;
    double distance = 0;
    
    double motorSpeed = 0.5;

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

        while(opModeIsActive())
        {
            changeVelo(lib);
            moveRobot(lib);
            turnRobot(lib);

            if(gamepad1.right_bumper)
            {
                carousel.setPower(-0.67);
            }
            else if(gamepad1.left_bumper)
            {
                carousel.setPower(0.67);
            }
            else
            {
                carousel.setPower(0);
            }

            if(gamepad1.right_trigger > 0.1)
            {
                outputRight.setVelocity(velocity);
                outputLeft.setVelocity(velocity);
            }
            else
            {
                outputRight.setVelocity(0);
                outputLeft.setVelocity(0);
            }

            if(gamepad1.triangleWasPressed())
            {
                ramp.setPower(0.5);
            }
            else if(gamepad1.crossWasPressed())
            {
                ramp.setPower(-0.5);
            }
            else
            {
                ramp.setPower(0);
            }

            telemetry.addData("Velocity: ", velocity);
            telemetry.addData("Distance (in): ", distance);
            telemetry.addData("TY: ", limelight.getLatestResult().getTy());
            telemetry.update();
        }
    }

    public void moveRobot(Library lib)
    {
        /* Checking if controller is going right
        Checks if x is on the right side (x is greater than 0)
        Checks if y is on the y-axis (y is between 0.5 and -0.5)
        */
        if (gamepad1.left_stick_x > 0 && (gamepad1.left_stick_y > -0.5 && gamepad1.left_stick_y < 0.5))
        {
            lib.driveRight(motorSpeed);
        }

        /* Checking if controller is going left
        Checks if x is on the left side (x is less than 0)
        Checks if y is on the y-axis (y is between 0.5 and -0.5)
        */
        else if (gamepad1.left_stick_x < 0 && (gamepad1.left_stick_y > -0.5 && gamepad1.left_stick_y < 0.5))
        {
            lib.driveLeft(motorSpeed);
        }
        /* Checking if controller is going up
        Checks if x is on the x-axis (x is between 0.5 and -0.5)
        Checks if y is on the up side (y is less than 0)
        */
        else if (gamepad1.left_stick_y < 0 && (gamepad1.left_stick_x > -0.5 && gamepad1.left_stick_x < 0.5))
        {
            lib.driveForward(motorSpeed);
        }
        /* Checking if controller is going down
        Checks if x is on the x-axis (x is between 0.5 and -0.5)
        Checks if y is on the bottom side (y is greater than 0)
        */
        else if (gamepad1.left_stick_y > 0 && (gamepad1.left_stick_x > -0.5 && gamepad1.left_stick_x < 0.5))
        {
            lib.driveBackward(motorSpeed);
        }
        /* Checking if controller is going up-right
        Checks if x is on the right side (x is greater than 0.5)
        Checks if y is on the top side (y is less than -0.5)
        */
        else if (gamepad1.left_stick_y < -0.5 && gamepad1.left_stick_x > 0.5)
        {
            lib.driveUpRight(motorSpeed);
        }
        /* Checking if controller is going up-left
        Checks if x is on the left side (x is less than -0.5)
        Checks if y is on the top side (y is less than -0.5)
        */
        else if (gamepad1.left_stick_y < -0.5 && gamepad1.left_stick_x < -0.5)
        {
            lib.driveUpLeft(motorSpeed);
        }
        /* Checking if controller is going down-right
        Checks if x is on the right side (x is greater than 0.5)
        Checks if y is on the bottom side (y is greater than 0.5)
        */
        else if (gamepad1.left_stick_y > 0.5 && gamepad1.left_stick_x > 0.5)
        {
            lib.driveDownRight(motorSpeed);
        }
        /* Checking if controller is going down-left
        Checks if x is on the left side (x is less than -0.5)
        Checks if y is on the bottom side (y is greater than 0.5)
        */
        else if (gamepad1.left_stick_y > 0.5 && gamepad1.left_stick_x < -0.5)
        {
            lib.driveDownLeft(motorSpeed);
        }
        /* If the controller is going nowhere else, it stops the robot
        Sets the power to all wheels to 0
        */
        else
        {
            lib.stopMoving();
        }
    }

    public void turnRobot(Library lib)
    {
        if(gamepad1.right_stick_x < -0.5)
        {
            lib.turnLeft(motorSpeed);
        }
        else if(gamepad1.right_stick_x > 0.5)
        {
            lib.turnRight(motorSpeed);
        }
        else
        {
            lib.stopMoving();
        }
    }

    public void changeVelo(Library lib)
    {
        limelight.pipelineSwitch(6);
        distance = lib.getAprilTagDistance(limelight.getLatestResult().getTy());

        if(limelight.getLatestResult().getTy() != 0)
        {
             velocity = 1077.678571 + (4.755952 * distance);
        }
        else
        {
            velocity = 1400;
        }
    }
}
