package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "VvhsTeleop")
public class VvhsTeleop extends LinearOpMode {


    Limelight3A limelight;
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor RearLeft;
    DcMotor RearRight;
    DcMotorEx outputRight;
    // Hasn't been tested yet
    DcMotorEx outputLeft;
    // Hasn't been tested yet.
    DcMotor intake;
    DigitalChannel touchSensorBot;
    DigitalChannel touchSensorTop;


    Servo gate;
    DcMotor ramp;
    CRServo carousel;
    boolean shootReady = false;
    double[] moveSpeeds = {0.3, 0.5, 0.7, 1};
    int speedIndex = 0;


    double motorSpeed = 1;
    double carouselPower = 0.67;

    double shooterP = 48.72995;
    double shooterI = 0;
    double shooterD = 0;
    double shooterF = 13.13319;
    double ShootingVelocity = 1267;

    double distance;
    @Override
    public void runOpMode()//initalizes all Motors and servos.
    {
        telemetry.addData("Initialize", "called");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");// MOTOR 0
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");// MOTOR 3
        RearLeft = hardwareMap.get(DcMotor.class, "RearLeft");// MOTOR 1
        RearRight = hardwareMap.get(DcMotor.class, "RearRight");// MOTOR 2


        outputRight = hardwareMap.get(DcMotorEx.class, "RightOutput");
        outputLeft = hardwareMap.get(DcMotorEx.class, "LeftOutput");


        outputRight.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        outputLeft.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);

        intake = hardwareMap.get(DcMotor.class, "Intake");


        carousel = hardwareMap.get(CRServo.class, "Carousel");
        //doesn't have a class yet
        ramp = hardwareMap.get(DcMotor.class, "rampIntakeOuttake");
        gate = hardwareMap.get(Servo.class, "gate");

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();


        // set motor needed position
        outputRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        telemetry.addData("Initial", "working");


        outputRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//
        outputLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        outputRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        telemetry.addData("Initial", "working");


        touchSensorTop = hardwareMap.get(DigitalChannel.class, "touchSensorTop");
        touchSensorTop.setMode(DigitalChannel.Mode.INPUT);


        touchSensorBot = hardwareMap.get(DigitalChannel.class, "touchSensorDown");
        touchSensorBot.setMode(DigitalChannel.Mode.INPUT);

        Library lib = new Library(FrontLeft, FrontRight, RearLeft, RearRight, this, outputRight, outputLeft, carousel, telemetry, limelight, intake, ramp, gate,colorSensor);

        while (opModeIsActive()) {
            ChangeMotorPowerSpeed();
            moveRobot();
            CarouselMovement();
            TurnRobot();
            LaunchMotors(ShootingVelocity);
            IntakeBalls();
            gateMovement();
            rampMovement();
            telemetry.update();
        }
    }


    public void ChangeMotorPowerSpeed()//changes MOTOR MOVEMENT Speed using M1 and M2
    {
        telemetry.addData("setPowerSpeed", "called");
        if (gamepad1.dpadUpWasPressed())//M1
        {
            telemetry.addData("dpad_up", "called");
            //If motor speed is less then 1 then increase by .1
            speedIndex += speedIndex < moveSpeeds.length - 1 ? 1 : 0;
            if (!(motorSpeed >= 0.9))
            {
                motorSpeed += 0.3;
                telemetry.addData("Motor Speed is : ", motorSpeed);
            }
        }
        if (gamepad1.dpadDownWasPressed())//M2
        {
            telemetry.addData("dpad_down", "called");
            //If motor speed is greater then -1 then decrease by .1
            speedIndex += speedIndex > 0 ? -1 : 0;
            telemetry.addData("Motor Speed is : ", motorSpeed);
        }
        motorSpeed = moveSpeeds[speedIndex];


        if (!(motorSpeed <= 0.3)) {
            motorSpeed -= 0.3;
        }
    }


    public void setMotorsPower(double fLSpeed, double fRSpeed, double rLSpeed, double rRSpeed)//function to set all motors to the same speed
    {
        FrontLeft.setPower(fLSpeed);
        FrontRight.setPower(fRSpeed);
        RearLeft.setPower(rLSpeed);
        RearRight.setPower(rRSpeed);
    }


    public void moveRobot() {
        telemetry.addData("joystick X: ", gamepad1.left_stick_x);
        telemetry.addData("joystick Y: ", gamepad1.left_stick_y);
        /* Checking if controller is going right
        Checks if x is on the right side (x is greater than 0)
        Checks if y is on the y-axis (y is between 0.5 and -0.5)
        */
        if (gamepad1.left_stick_x > 0 && (gamepad1.left_stick_y > -0.5 && gamepad1.left_stick_y < 0.5)) {
            setMotorsPower(-motorSpeed, -motorSpeed, motorSpeed, motorSpeed);
            telemetry.addData("Direction: ", "Right");
        }

        /* Checking if controller is going left
        Checks if x is on the left side (x is less than 0)
        Checks if y is on the y-axis (y is between 0.5 and -0.5)
        */
        else if (gamepad1.left_stick_x < 0 && (gamepad1.left_stick_y > -0.5 && gamepad1.left_stick_y < 0.5)) {
            setMotorsPower(motorSpeed, motorSpeed, -motorSpeed, -motorSpeed);
            telemetry.addData("Direction: ", "Left");
        }
        /* Checking if controller is going up
        Checks if x is on the x-axis (x is between 0.5 and -0.5)
        Checks if y is on the up side (y is less than 0)
        */
        else if (gamepad1.left_stick_y < 0 && (gamepad1.left_stick_x > -0.5 && gamepad1.left_stick_x < 0.5)) {
            setMotorsPower(-motorSpeed, motorSpeed, -motorSpeed, motorSpeed);
            telemetry.addData("Direction: ", "Up");
        }
        /* Checking if controller is going down
        Checks if x is on the x-axis (x is between 0.5 and -0.5)
        Checks if y is on the bottom side (y is greater than 0)
        */
        else if (gamepad1.left_stick_y > 0 && (gamepad1.left_stick_x > -0.5 && gamepad1.left_stick_x < 0.5)) {
            setMotorsPower(motorSpeed, -motorSpeed, motorSpeed, -motorSpeed);
            telemetry.addData("Direction: ", "Down");
        }
        /* Checking if controller is going up-right
        Checks if x is on the right side (x is greater than 0.5)
        Checks if y is on the top side (y is less than -0.5)
        */
        else if (gamepad1.left_stick_y < -0.5 && gamepad1.left_stick_x > 0.5) {
            setMotorsPower(-motorSpeed, 0, 0, motorSpeed);
            telemetry.addData("Direction: ", "Up-Right");
        }
        /* Checking if controller is going up-left
        Checks if x is on the left side (x is less than -0.5)
        Checks if y is on the top side (y is less than -0.5)
        */
        else if (gamepad1.left_stick_y < -0.5 && gamepad1.left_stick_x < -0.5) {
            setMotorsPower(0, motorSpeed, -motorSpeed, 0);
            telemetry.addData("Direction: ", "Up-Left");
        }
        /* Checking if controller is going down-right
        Checks if x is on the right side (x is greater than 0.5)
        Checks if y is on the bottom side (y is greater than 0.5)
        */
        else if (gamepad1.left_stick_y > 0.5 && gamepad1.left_stick_x > 0.5) {
            setMotorsPower(0, -motorSpeed, motorSpeed, 0);
            telemetry.addData("Direction: ", "Down-Right");
        }
        /* Checking if controller is going down-left
        Checks if x is on the left side (x is less than -0.5)
        Checks if y is on the bottom side (y is greater than 0.5)
        */
        else if (gamepad1.left_stick_y > 0.5 && gamepad1.left_stick_x < -0.5) {
            setMotorsPower(motorSpeed, 0, 0, -motorSpeed);
            telemetry.addData("Direction: ", "Down-Left");
        }
        /* If the controller is going nowhere else, it stops the robot
        Sets the power to all wheels to 0
        */
        else
        {
            FrontLeft.setPower(0.0);
            FrontRight.setPower(0.0);
            RearLeft.setPower(0.0);
            RearRight.setPower(0.0);
            telemetry.addData("Direction: ", "Still");
        }
    }

    public void TurnRobot()
    {
        if (gamepad1.right_stick_x > 0)
        {
            telemetry.addData("joystick X:", gamepad1.right_stick_x);
            setMotorsPower(-motorSpeed, -motorSpeed, -motorSpeed, -motorSpeed);
        }
        if (gamepad1.right_stick_x < 0)
        {
            telemetry.addData("joystick X:", gamepad1.right_stick_x);
            setMotorsPower(motorSpeed, motorSpeed, motorSpeed, motorSpeed);
        }
    }

    public void LaunchMotors(double outputMotorVelocity)
    {
        //outputRight.setVelocity(2800);
        if (gamepad2.right_trigger > 0.4)
        {

            telemetry.addData("Shooting velo", outputRight.getVelocity());
            telemetry.addData("Shooting Motor Speed", outputRight.getPower());
            outputRight.setVelocity(outputMotorVelocity);
            outputLeft.setVelocity(outputMotorVelocity);
//            sleep(2500);

        }
        else
        {
            outputRight.setVelocity(0.0);
            outputLeft.setVelocity(0.0);
        }
    }

    public void IntakeBalls()
    {
        if (gamepad2.left_trigger > 0.1 || gamepad1.left_trigger > 0.1)
        {
            intake.setPower(1);
        }
        else if(gamepad1.right_trigger > 0.1)
        {
            intake.setPower(-1);
        }
        else
        {
            intake.setPower(0);
        }
    }


    public void CarouselMovement()
    {
        if (gamepad2.right_bumper || gamepad1.right_bumper)
        {
            carousel.setPower(-carouselPower);
            telemetry.addData("Carousel Power", String.valueOf(carouselPower));
        }
        else if (gamepad2.left_bumper || gamepad1.left_bumper)
        {
            carousel.setPower(carouselPower);
            telemetry.addData("Carousel Power", String.valueOf(carouselPower));
        }
        else
        {
            carousel.setPower(0.0);
            telemetry.addData("Carousel Power", "0.0");
        }
    }


    public void rampMovement() {
        int currentPositionOfRamp = ramp.getCurrentPosition();
        telemetry.addData("Ramp Position: ", currentPositionOfRamp);
        telemetry.addData("top sensor", touchSensorTop.getState());
        telemetry.addData("bottom sensor", touchSensorBot.getState());
        ramp.setPower(0);
        if (gamepad2.triangle || gamepad1.triangle)
        {
            ramp.setPower(0.5);
            intake.setPower(1);
            telemetry.addData("Ramp Status: ", "Going UP");
            if (!touchSensorTop.getState())
            {
                gamepad2.rumble(1, 1, 500);
                gamepad1.rumble(1, 1, 500);
            }
        }
        else
        {
            ramp.setPower(0.0);
        }
        if (gamepad2.cross || gamepad1.cross)
        {
            ramp.setPower(-0.3);
            intake.setPower(-1);
            telemetry.addData("Ramp Status: ", " Going DOWN");
            if (!touchSensorBot.getState())
            {

                gamepad2.rumble(1, 1, 500);
                gamepad1.rumble(1, 1, 500);
            }
        }
        else
        {
            ramp.setPower(0);
            intake.setPower(0);
            telemetry.addData("Ramp Status: ", "Not moving");
        }

//        if(gamepad2.)
    }

    public void gateMovement()
    {
        double gatePosition = gate.getPosition();
        if (gamepad2.square || gamepad1.square)
        {
            gate.setPosition(gatePosition + 0.02);
            telemetry.addData("Gate Status: ",  "OPENING");
        }
        if (gamepad2.circle || gamepad1.circle)
        {
            gate.setPosition(gatePosition - 0.02);
            telemetry.addData("Gate Status: ", "CLOSING");
        }
        telemetry.addData("Gate Position:", gate.getPosition());
    }



}