package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.InitOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "TeleBlue", group = "OpModes")
public class TeleBlue extends InitOpMode {



    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);
        aprilTagWebcam.displayDetectionTelemetry(id24);


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
        if (gamepad1.dpadUpWasPressed()) { // motors have 28 ticks per revolution
            DetectedAprilTagSpeed = aprilTagWebcam.calculateVelocity(id24);
            if (DetectedAprilTagSpeed == -1.0) {
                if (FlywheelSpeed < 1500.00) {
                    FlywheelSpeed += (1500.0 * (1.0 / 6.0));
                } else if (FlywheelSpeed >= 1500.0) {
                    FlywheelSpeed = 1500.0;
                }
            } else if (DetectedAprilTagSpeed != -1.0 && DetectedAprilTagSpeed > 0.0){
                FlywheelSpeed = DetectedAprilTagSpeed;
            }
        }
        if (gamepad1.dpadDownWasPressed()) {
            FlywheelSpeed = 0.0;
        }
        Flywheel.setVelocity(FlywheelSpeed);
        telemetry.addData("Target Flywheel Speed:", FlywheelSpeed);
        telemetry.addData("Actual Flywheel Speed:", Flywheel.getVelocity());
        if (gamepad1.right_trigger > 0.0) {
            //Intaking ON
            Intake.setPower(1);
        } else if (gamepad1.right_trigger == 0.0) {
            //Intaking OFF
            Intake.setPower(0);
        }

        if (gamepad1.left_trigger > 0.0) {
            //Ejection ON
            Intake.setPower(-1);
        } else if (gamepad1.right_trigger == 0.0) {
            //Ejection OFF
            Intake.setPower(0);
        }


        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        colors = colorSensor.getNormalizedColors();
        float normred, normgreen, normblue;
        normred = colors.red/colors.alpha;
        normgreen = colors.green/colors.alpha;
        normblue = colors.blue/colors.alpha;

        telemetry.addData("Red", normred);
        telemetry.addData("Green", normgreen);
        telemetry.addData("Blue", normblue);
        telemetry.addData("Intake Power: " , Intake.getPower());

        telemetry.update();
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

