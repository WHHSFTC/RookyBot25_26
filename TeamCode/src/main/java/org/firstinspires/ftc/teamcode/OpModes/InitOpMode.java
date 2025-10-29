package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class InitOpMode extends OpMode {
    protected Drivetrain drivetrain;
    protected Gamepad gamepad1 = new Gamepad();

    // This method runs once when the "INIT" button is pressed on Driver Station
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, gamepad1);

    }

    public void loop() {

    }
}
