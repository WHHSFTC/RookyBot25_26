package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.InitOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TeleBlue", group = "OpModes")
public class TeleBlue extends InitOpMode {

boolean intakeState = false;
    @Override
    public void loop() {


        if (gamepad1.right_bumper) {
            sideFlipperRight.setPosition(closedFlipperPosition);
        } else {
            sideFlipperRight.setPosition(openFlipperPosition);
        }
        if (gamepad1.left_bumper) {
            sideFlipperLeft.setPosition(openFlipperPosition);
        } else {
            sideFlipperLeft.setPosition(closedFlipperPosition);
        }
        if (gamepad1.dpad_right) {
            middleFlipper.setPosition(middleFlipperPosition);
        }
        if (gamepad1.dpad_left) {
            middleFlipper.setPosition(-middleFlipperPosition);
        }
        if (gamepad1.dpad_up) {
            if (FlywheelVelocity < 2800.0) {
                FlywheelVelocity = FlywheelVelocity + (2800.0 * (1.0/6.0));
            } else if (FlywheelVelocity >= 2800.0) {
                FlywheelVelocity = 2800.0;
            }
            Flywheel.setVelocity(FlywheelVelocity);
            telemetry.addData("New Robot Velocity: ", Flywheel.getVelocity());
            telemetry.update();
        }
        if (gamepad1.dpad_down) {
            Flywheel.setVelocity(0);
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

