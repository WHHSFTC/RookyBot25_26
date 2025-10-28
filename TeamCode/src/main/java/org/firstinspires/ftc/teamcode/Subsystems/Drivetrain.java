package org.firstinspires.ftc.teamcode.Subsystems; //Since subsystems are in a different folder, they must be "packaged" for use in other folders

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

public class Drivetrain {
    // Fields
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private boolean turtleMode; // Turtle Mode is our team's lingo for slow mode, set to TRUE to slow down robot
    private double turtleMultiplier; // This number should represent the final percent speed wanted. If 60% is desired, then 0.6
    private double frontLeftPower, frontRightPower, backLeftPower, backRightPower; // Declared here to allow us to have a getter for telemetry
    private Gamepad driverOneGamepad;

    // Constructor: Initialize motors using hardwareMap
    public Drivetrain(HardwareMap hardwareMap, Gamepad gamepad) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        driverOneGamepad = gamepad;

        // Reverse motors if needed
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        turtleMode = false; // Default turtleMode is off
        turtleMultiplier = 0.6; // Default 60% multiplier
    }

    public void moveWithGamepad() {
        double x, y, rx;
        y = -driverOneGamepad.left_stick_y; // Y stick reversed
        x = driverOneGamepad.left_stick_x;
        rx = driverOneGamepad.right_stick_x;

        drive(x, y, rx);
    }

    // Method to drive the robot using mecanum wheel calculations
    // y: Forward/backward movement
    // x: Left/right movement (strafing)
    // rotation: Turning in place
    private void drive(double x, double y, double rotation) {
        frontLeftPower = y + x + rotation;
        frontRightPower = y - x - rotation;
        backLeftPower = y - x + rotation;
        backRightPower = y + x - rotation;

        // Find the maximum power value
        double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        // Scale motor powers down if they exceed 1.0
        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;

        // Apply turtle multiplier if turtle mode is enabled
        if (turtleMode) {
            frontLeftPower *= turtleMultiplier;
            frontRightPower *= turtleMultiplier;
            backLeftPower *= turtleMultiplier;
            backRightPower *= turtleMultiplier;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    // Getters and Setters
    public void setTurtleMode(boolean mode) {
        turtleMode = mode;
    }

    public void setTurtleMultiplier(double multiplier) {
        turtleMultiplier = multiplier;
    }

    public boolean isTurtleMode() {
        return turtleMode;
    }

    public double getTurtleMultiplier() {
        return turtleMultiplier;
    }

    public double getFrontLeftPower() {
        return frontLeftPower;
    }

    public double getFrontRightPower() {
        return frontRightPower;
    }

    public double getBackLeftPower() {
        return backLeftPower;
    }

    public double getBackRightPower() {
        return backRightPower;
    }

    // Method to stop the motors
    public void stop() {
        drive(0, 0, 0);
    }
}
