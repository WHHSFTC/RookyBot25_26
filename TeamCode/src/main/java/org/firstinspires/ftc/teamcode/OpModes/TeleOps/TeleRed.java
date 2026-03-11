package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.OpModes.InitOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
@TeleOp(name = "TeleRed", group = "OpModes")
public class TeleRed extends InitOpMode {
    private TelemetryManager telemetryM;

    //PID Tuning
    static double targetTPS;
    static double kP = 0.75;
    static double kI = 0;
    static double kD = 0;
    static double kF = 15;

    @Override
    public void loop() {
        MeasuredTPS = Flywheel.getVelocity();

        Flywheel.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.debug("targetTPS", targetTPS);
        telemetryM.debug("measuredTPS", MeasuredTPS);

        telemetryM.update(telemetry);

        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);;
        aprilTagWebcam.displayDetectionTelemetry(id24);

        DetectedAprilTagSpeed = aprilTagWebcam.calculateVelocity(id24);

        if (FlywheelStatus) {
            if (DetectedAprilTagSpeed != -1.0 && DetectedAprilTagSpeed > 0.0) {
                targetTPS = DetectedAprilTagSpeed;
                ManualFlywheelMode = false;
            }
        }

        if (BrakingStatus) {
            if (MeasuredTPS > 100) {
                targetTPS = -100.0;
            } else {
                targetTPS = 0.0;
                BrakingStatus = false;
            }
        }

        if (FlywheelStatus) {
            if (MeasuredTPS < (targetTPS + 50) && MeasuredTPS > (targetTPS - 50)) {
                gamepad1.rumble(200);
            }
        }

        if (gamepad1.dpadUpWasPressed()) { // motors have 28 ticks per revolution
            FlywheelStatus = true;
            if (ManualFlywheelMode == false) {
                targetTPS = 0.0;
            }
            if (DetectedAprilTagSpeed == -1.0) {
                ManualFlywheelMode = true;
                if (targetTPS < 1500.00) {
                    targetTPS += (1500.0 * (1.0 / 6.0));
                } else if (FlywheelSpeed >= 1500.0) {
                    targetTPS = 1500.0;
                }
            }
        }

        if (gamepad1.dpadDownWasPressed()) {
            FlywheelStatus = false;
            BrakingStatus = true;
        }

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
        Flywheel.setVelocity(targetTPS);
    }

    public void mecanumDrive(float v, float leftStickX, double rightStickX) {
        double y = v; // Remember, Y stick is reversed!
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

