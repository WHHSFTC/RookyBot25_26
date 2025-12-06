package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class InitOpMode extends OpMode {
    public Servo sideFlipperLeft, sideFlipperRight, middleFlipper;
    public DcMotorEx Flywheel,Intake;
    public Double openFlipperPosition = 0.4;
    public Double closedFlipperPosition = 0.0;
    public Double middleFlipperPosition = 0.22;
    public Double FlywheelVelocity = 0.0;
    public DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;

    @Override
    public void init() {
        sideFlipperLeft = hardwareMap.get(Servo.class, "sideFlipperLeft");
        sideFlipperRight = hardwareMap.get(Servo.class, "sideFlipperRight");
        middleFlipper = hardwareMap.get(Servo.class, "middleFlipper");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        //DRive train motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        //Flippers
        sideFlipperRight.setPosition(openFlipperPosition);
        sideFlipperLeft.setPosition(closedFlipperPosition);
        middleFlipper.setPosition(closedFlipperPosition);
        //Drivetrain forward and backwards
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop() {

    }
}
