package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Subsystems.AprilTagWebcam;

public class InitOpMode extends OpMode {
    public DcMotorEx Flywheel, Intake, Turret;
    public Double FlywheelSpeed = 0.0;
    public Double DetectedAprilTagSpeed;
    public Double AdjustmentSpeed;
    public Double aprilTagYaw;
    public boolean FlywheelStatus;
    public boolean ManualFlywheelMode = false;
    public DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    public Double Speed;
    public NormalizedColorSensor colorSensor;
    public boolean BrakingStatus;
    public Double MeasuredTPS;
    //public ElapsedTime actionTimer1 = new ElapsedTime();
    public int intakeState1 = 0;
    // public ElapsedTime actionTimer = new ElapsedTime();
    public int intakeState = 0;

    // public NormalizedRGBA colors;

    public AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        Flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);





        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        //Drive train motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        //Drivetrain forward and backwards
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop() {

    }
}
