package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Output {
    // Motor
    private final DcMotorEx flywheelMotor;

    // Servos
    private final Servo hoodServo;
    private final Servo transferServo;

    // Servo Positions - TUNE THESE VALUES
    public static final double ARTIFACT_FEEDER_FEED_POSITION = 0.5;
    public static final double ARTIFACT_FEEDER_RESET_POSITION = 0.0;
    // TUNE THIS VALUE: The time (in ms) the feeder stays in the "feed" position to launch an artifact.
    public static long ARTIFACT_FEED_DURATION_MS = 250;

    // PID Coefficients
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0; // Feedforward for velocity control

    // RPM variables
    private double targetRPM;
    private double currentRPM;

    // Motor specifics for Gobilda 5203 6000rpm motor
    private static final double TICKS_PER_REVOLUTION = 28;
    private static final double GEAR_RATIO = 1.0; // If there's a gearbox

    // RPM tolerance
    private static final double RPM_TOLERANCE = 20.0;

    // Constructor
    public Output(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        transferServo = hardwareMap.get(Servo.class, "transferServo");


        // It's good practice to reset the encoder
        flywheelMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set PID coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, kI, kD, kF);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    /**
     * Sets the flywheel motor's target RPM and returns whether it's at the target speed.
     * @param targetRPM The desired RPM for the flywheel.
     * @return true if the motor is at the target RPM, false otherwise.
     */
    public boolean setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;

        // Convert RPM to ticks per second
        double ticksPerSecond = targetRPM * TICKS_PER_REVOLUTION * GEAR_RATIO / 60.0;

        // Set the motor's target velocity
        flywheelMotor.setVelocity(ticksPerSecond);

        // Get current velocity in ticks per second
        double currentTicksPerSecond = flywheelMotor.getVelocity();

        // Convert current ticks per second to RPM
        this.currentRPM = currentTicksPerSecond * 60.0 / (TICKS_PER_REVOLUTION * GEAR_RATIO);

        // Check if the current RPM is within the tolerance
        return Math.abs(this.currentRPM - this.targetRPM) < RPM_TOLERANCE;
    }

    // Servo Methods
    /**
     * Feeds an artifact into the flywheel and then resets the servo.
     * Note: This method uses Thread.sleep(), which will pause the entire program.
     * Use with caution in a fast control loop.
     */
    public void feedArtifact() {
        transferServo.setPosition(ARTIFACT_FEEDER_FEED_POSITION);
        try {
            Thread.sleep(ARTIFACT_FEED_DURATION_MS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        transferServo.setPosition(ARTIFACT_FEEDER_RESET_POSITION);
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    // Getters
    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public PIDFCoefficients getPIDFCoefficients() {
        return flywheelMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public double getHoodPosition(){
        return hoodServo.getPosition();
    }

    public double getTransferPosition(){
        return transferServo.getPosition();
    }

    public long getArtifactFeedDuration() {
        return ARTIFACT_FEED_DURATION_MS;
    }

    // Setters
    public void setPIDFCoefficients(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        PIDFCoefficients newCoefficients = new PIDFCoefficients(kP, kI, kD, kF);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, newCoefficients);
    }

    public void setArtifactFeedDuration(long duration) {
        ARTIFACT_FEED_DURATION_MS = duration;
    }
}
