package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModes.InitOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import com.qualcomm.robotcore.hardware.Gamepad;
@TeleOp(name = "TeleBlue", group = "OpModes")
public class TeleBlue extends InitOpMode {
    Servo sideFlipperLeft;
    Double openFlipperPosition = 0.33;
    Double closedFlipperPosition = 0.0;
    @Override
    public void loop() {
        sideFlipperLeft = hardwareMap.get(Servo.class, "sideFlipperLeft");
        sideFlipperLeft.setPosition(Math.abs(gamepad1.left_stick_x));
        telemetry.addData("Servo Postiion:", sideFlipperLeft.getPosition());

    }
}
