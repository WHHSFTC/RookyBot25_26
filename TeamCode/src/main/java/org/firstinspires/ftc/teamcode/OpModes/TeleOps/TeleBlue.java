package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.InitOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp(name = "TeleBlue", group = "OpModes")
public class TeleBlue extends InitOpMode {

    private ElapsedTime pidTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private boolean isAligning = false;
    
    // PID Coefficients
    public static double Kp = 0.05; // Reduced Kp to prevent oscillation
    public static double Ki = 0.0;
    public static double Kd = 0.0;

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

        DetectedAprilTagSpeed = aprilTagWebcam.calculateVelocity(id24);

        if (FlywheelStatus) {
            if (DetectedAprilTagSpeed != -1.0 && DetectedAprilTagSpeed > 0.0) {
                FlywheelSpeed = DetectedAprilTagSpeed;
                ManualFlywheelMode = false;
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

        if (BrakingStatus) {
            if (CurrentFlywheelSpeed > 100) {
                FlywheelSpeed = -100.0;
            } else {
                FlywheelSpeed = 0.0;
                BrakingStatus = false;
            }
        }

        if (gamepad1.dpadDownWasPressed()) {
            FlywheelStatus = false;
            FlywheelSpeed = 0.0;
        }
        Flywheel.setVelocity(FlywheelSpeed);
        telemetry.addData("Target Flywheel Speed:", FlywheelSpeed);
        telemetry.addData("Actual Flywheel Speed:", Flywheel.getVelocity());
        CurrentFlywheelSpeed = Flywheel.getVelocity();
        
        if (gamepad1.right_trigger > 0.0) {
            //Intaking ON
            Intake.setPower(1);
        } else if (gamepad1.left_trigger > 0.0) {
            //Ejection ON
            Intake.setPower(-1);
        } else {
            //Intaking/Ejection OFF
            Intake.setPower(0);
        }

        // Toggle alignment mode
        if (gamepad1.aWasPressed()) {
            isAligning = !isAligning;
            if (isAligning) {
                pidTimer.reset();
                integralSum = 0;
                lastError = 0;
            }
        }
        
        double turn = gamepad1.right_stick_x;

        if (isAligning) {
            aprilTagYaw = aprilTagWebcam.getAprilTagYaw(id24);

            if (aprilTagYaw != -1000) {
                // Target yaw is 0. The robot will rotate to face the tag.
                turn = -PIDControl(0, aprilTagYaw);
            }
        }


        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, (float)turn);

        colors = colorSensor.getNormalizedColors();
        float normred, normgreen, normblue;
        normred = colors.red/colors.alpha;
        normgreen = colors.green/colors.alpha;
        normblue = colors.blue/colors.alpha;

        telemetry.addData("Red", normred);
        telemetry.addData("Green", normgreen);
        telemetry.addData("Blue", normblue);
        telemetry.addData("Intake Power: " , Intake.getPower());
        telemetry.addData("Aligning", isAligning);
        telemetry.addData("AprilTag Yaw", aprilTagYaw);
        telemetry.addData("Turn Power", turn);

        telemetry.update();
    }

    public double PIDControl(double target, double current) {
        double error = target - current;
        double dt = pidTimer.seconds();
        pidTimer.reset();
        
        // Prevent division by zero
        if (dt < 1e-6) dt = 1e-6;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }

    public void mecanumDrive(float v, float leftStickX, float rightStickX) {
        double y = v; // Using passed parameters
        double x = leftStickX;
        double rx = rightStickX;

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
