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
        } else if (intakeState1 == 0){
            sideFlipperRight.setPosition(openFlipperPosition);
        }
        if (gamepad1.left_bumper) {
            sideFlipperLeft.setPosition(openFlipperPosition);
        } else if (intakeState == 0){
            sideFlipperLeft.setPosition(closedFlipperPosition);
        }
        if (gamepad1.dpad_right) {
            middleFlipper.setPosition(middleFlipperPosition);
        }
        if (gamepad1.dpad_left) {
            middleFlipper.setPosition(-middleFlipperPosition);
        }

        DetectedAprilTagSpeed = aprilTagWebcam.calculateVelocity(id24);

        if (FlywheelStatus) {
            if (DetectedAprilTagSpeed != -1.0 && DetectedAprilTagSpeed > 0.0) {
                FlywheelSpeed = DetectedAprilTagSpeed;
                ManualFlywheelMode = false;
            }
        }

        if (BrakingStatus) {
            if (CurrentFlywheelSpeed > 100) {
                FlywheelSpeed = -100.0;
            } else {
                FlywheelSpeed = 0.0;
                BrakingStatus = false;
            }
        }

        if (gamepad1.dpadUpWasPressed()) { // motors have 28 ticks per revolution
            FlywheelStatus = true;
            if (ManualFlywheelMode == false) {
                FlywheelSpeed = 0.0;
            }
            if (DetectedAprilTagSpeed == -1.0) {
                ManualFlywheelMode = true;
                if (FlywheelSpeed < 1500.00) {
                    FlywheelSpeed += (1500.0 * (1.0 / 6.0));
                } else if (FlywheelSpeed >= 1500.0) {
                    FlywheelSpeed = 1500.0;
                }
            }
        }

        if (gamepad1.dpadDownWasPressed()) {
            FlywheelStatus = false;
            BrakingStatus = true;
        }
        Flywheel.setVelocity(FlywheelSpeed);
        telemetry.addData("Target Flywheel Speed:", FlywheelSpeed);
        telemetry.addData("Actual Flywheel Speed:", Flywheel.getVelocity());
        CurrentFlywheelSpeed = Flywheel.getVelocity();
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

        if (gamepad1.a) {
            aprilTagYaw = aprilTagWebcam.getAprilTagYaw(id24);
            AdjustmentSpeed = aprilTagWebcam.getAdjustmentSpeed(id24);


            if (aprilTagYaw == -1000) {return;}
            if (aprilTagYaw > 0.5) { // robot is angled too far right; robot needs to turn left
                leftFrontMotor.setPower(-AdjustmentSpeed);
                leftBackMotor.setPower(-AdjustmentSpeed);
                rightFrontMotor.setPower(AdjustmentSpeed);
                rightBackMotor.setPower(AdjustmentSpeed);
            } else if (aprilTagYaw < -0.5) { // robot is angled too far left; robot needs to turn right
                leftFrontMotor.setPower(AdjustmentSpeed);
                leftBackMotor.setPower(AdjustmentSpeed);
                rightFrontMotor.setPower(-AdjustmentSpeed);
                rightBackMotor.setPower(-AdjustmentSpeed);
            }

        }
        if (gamepad1.bWasPressed() && intakeState1 == 0) {
            Intake.setPower(1.0);   // step 1
            actionTimer1.reset();
            intakeState1 = 1;
        }

        switch (intakeState1) {
            case 1:
                Intake.setPower(1.0); // keep intake ON
                if (actionTimer1.seconds() >= 0.09) {
                    sideFlipperRight.setPosition(closedFlipperPosition);
                    actionTimer1.reset();
                    intakeState1 = 2;
                }
                break;

            case 2:
                Intake.setPower(1.0); // keep intake ON until sequence finishes
                if (actionTimer1.seconds() >= 0.45) {
                    sideFlipperRight.setPosition(openFlipperPosition);
                    Intake.setPower(0.0); // now turn off
                    intakeState1 = 0;       // reset for next press
                }
                break;
        }

        if (gamepad1.xWasPressed() && intakeState == 0) {
            Intake.setPower(1.0);   // step 1
            actionTimer.reset();
            intakeState = 1;
        }

        switch (intakeState) {
            case 1:
                Intake.setPower(1.0); // keep intake ON
                if (actionTimer.seconds() >= 0.09) {
                    sideFlipperLeft.setPosition(openFlipperPosition);
                    actionTimer.reset();
                    intakeState = 2;
                }
                break;

            case 2:
                Intake.setPower(1.0); // keep intake ON until sequence finishes
                if (actionTimer.seconds() >= 0.45) {
                    sideFlipperLeft.setPosition(closedFlipperPosition);
                    Intake.setPower(0.0); // now turn off
                    intakeState = 0;       // reset for next press
                }
                break;
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

