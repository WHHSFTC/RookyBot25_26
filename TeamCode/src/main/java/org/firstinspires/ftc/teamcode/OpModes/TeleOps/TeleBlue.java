package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModes.InitOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TeleBlue", group = "OpModes")
public class TeleBlue extends InitOpMode {

boolean intakeState = false;
    @Override
    public void loop() {


        if (gamepad1.right_bumper) {
            sideFlipperRight.setPosition(openFlipperPosition);
        } else {
            sideFlipperRight.setPosition(closedFlipperPosition);
        }
        if (gamepad1.left_bumper) {
            sideFlipperLeft.setPosition(closedFlipperPosition);
        } else {
            sideFlipperLeft.setPosition(openFlipperPosition);
        }
        if (gamepad1.dpad_right) {
            middleFlipper.setPosition(middleFlipperPosition);
        }
        if (gamepad1.dpad_left) {
            middleFlipper.setPosition(-middleFlipperPosition);
        }
        if (gamepad1.dpad_up) {
          FlywheelSpeed= FlywheelSpeed + 1000.0;
          Flywheel.setPower(FlywheelSpeed);
        } else if (gamepad1.dpad_down) {
           Flywheel.setPower(0);
        }
        if (gamepad1.dpad_up) {
            if (FlywheelSpeed < 6000.0) {
                FlywheelSpeed = FlywheelSpeed + 1000.0;
            } else if (FlywheelSpeed >= 6000.0) {
                FlywheelSpeed = 6000.0;
            }
            Flywheel.setPower(FlywheelSpeed);
        }
        if (gamepad1.dpad_down) {
            Flywheel.setPower(0);
        }
        if (gamepad1.a && intakeState == false) {
            //Code for when intake is turning on
            intakeState = true;
            Intake.setPower(1150);
        } else if (gamepad1.a && intakeState == true) {
            //Code for when intake is turning off
            intakeState = false;
            Intake.setPower(0);
        }

        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    public void mecanumDrive(float v, float leftStickX, float rightStickX) {
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontMotor.setPower(frontLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        leftBackMotor.setPower(backLeftPower);
        rightBackMotor.setPower(backRightPower);
    }

}

