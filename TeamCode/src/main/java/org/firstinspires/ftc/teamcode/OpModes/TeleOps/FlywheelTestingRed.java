package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.OpModes.InitOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
@TeleOp(name = "FlywheelTestingRed", group = "OpModes")
public class FlywheelTestingRed extends InitOpMode {
    private TelemetryManager telemetryM;

    //PID Tuning
    static double targetTPS;
    static double kP = 0.75;
    static double kI = 0;
    static double kD = 0;
    static double kF = 15;

    double targetAdjustmentSpeed;
    boolean turningStatus = false;

    double currentAngle;
    double AdjustmentCount;

    private double AprilTagBearing  = -1000;

    boolean TurretStatus = false;


    @Override
    public void loop() {

        // USE ENCODER FOR ANGLE TRACKING AND LIMITING OFF LIMIT ANGLES BUT USE SETPOWER OR SETVELOCITY IDK FOR ACTUALLY MOVING THE TURRET
        currentAngle = (Turret.getCurrentPosition() / 1300.0) * 360.0;
        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);
        AprilTagBearing = aprilTagWebcam.getAprilTagBearing(id24);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        if (TurretStatus) {

            if (AprilTagBearing != -1000.0) {
                if (AprilTagBearing > 1) {
                    AdjustmentCount++;
                    targetAdjustmentSpeed = (Math.pow(AprilTagBearing, 2) * 0.0015) + 0.17;
                    turningStatus = true;
                } else if (AprilTagBearing < -1) {
                    AdjustmentCount++;
                    targetAdjustmentSpeed = (Math.pow(AprilTagBearing, 2) * -0.0015) + 0.017;
                    turningStatus = true;
                } else if (AprilTagBearing > -1 && AprilTagBearing < 3) {
                    targetAdjustmentSpeed = 0;
                    turningStatus = false;
                }

            } else {
                targetAdjustmentSpeed = 0;
                turningStatus = false;
            }
        }

        if (currentAngle > 30 || currentAngle < -220) {
            targetAdjustmentSpeed = 0;
            turningStatus = false;
        }

        Turret.setPower(targetAdjustmentSpeed);
        telemetryM.debug("AprilTagBearing:", AprilTagBearing);
        telemetryM.debug("Adjustment Count:", AdjustmentCount                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    );
        telemetryM.debug("Turret Adjustment Speed:", targetAdjustmentSpeed);

        if (gamepad1.aWasPressed()) {
            if (TurretStatus == false) {
                TurretStatus = true;
            } else if (TurretStatus == true) {
                TurretStatus = false;
                Turret.setPower(0);
            }
        }

        if (gamepad1.x) {
            Turret.setPower(0.3);
        } else if (gamepad1.b) {
            Turret.setPower(-0.3);
        } else {
            Turret.setPower(0);
        }


        MeasuredTPS = Flywheel.getVelocity();

        Flywheel.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );

        telemetryM.debug("TurretStatus", TurretStatus);
        telemetryM.debug("TurretAngle", currentAngle);

        telemetryM.debug("targetTPS", targetTPS);
        telemetryM.debug("measuredTPS", MeasuredTPS);


        telemetryM.update(telemetry);

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
            ManualFlywheelMode = true;
            if (targetTPS < 1600.00) {
                targetTPS += (1600.00 * (1.0 / 8.0));
            } else if (FlywheelSpeed >= 1600.00) {
                targetTPS = 1600.00;
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

