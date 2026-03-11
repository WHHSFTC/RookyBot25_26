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
@TeleOp(name = "PIDTuning", group = "OpModes")
public class PIDTuning extends InitOpMode {


    private TelemetryManager telemetryM;

    //PID Tuning
    public static double targetTPS;
    public static double kP;
    public static double kI;
    public static double kD;
    public static double kF;



    @Override
    public void loop() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.debug("targetTPS", targetTPS);
        telemetryM.debug("measuredTPS", Flywheel.getVelocity());

        telemetryM.debug("kP", kP);
        telemetryM.debug("kI", kI);
        telemetryM.debug("kD", kD);
        telemetryM.debug("kF", kF);

        telemetryM.update(telemetry);

        Flywheel.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );



        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);;
        aprilTagWebcam.displayDetectionTelemetry(id24);

        DetectedAprilTagSpeed = aprilTagWebcam.calculateVelocity(id24);

        if (FlywheelStatus) {
            if (DetectedAprilTagSpeed != -1.0 && DetectedAprilTagSpeed > 0.0) {
                FlywheelSpeed = DetectedAprilTagSpeed;
                ManualFlywheelMode = false;
            }
        }

        if (BrakingStatus) {
            if (MeasuredTPS > 100) {
                FlywheelSpeed = -100.0;
            } else {
                FlywheelSpeed = 0.0;
                BrakingStatus = false;
            }
        }

        if (FlywheelStatus) {
            if (MeasuredTPS < (FlywheelSpeed + 50) && MeasuredTPS > (FlywheelSpeed - 50)) {
                gamepad1.rumble(200);
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
        Flywheel.setVelocity(targetTPS);
        MeasuredTPS = Flywheel.getVelocity();

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

