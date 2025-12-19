package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;
    public List<AprilTagDetection> detectedTags = new ArrayList<>();

    public Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {return;}
        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }
    private double aprilTagYaw;
    private double aprilTagYDistanceAdjusting;
    private double aprilTagYDistanceVelocity;
    private double RotateSpeed;
    public double getAprilTagYaw(AprilTagDetection detectedId) {
        if (detectedId == null) {
            return -1000;
        }
        if (detectedId.metadata != null) {
            aprilTagYaw = detectedId.ftcPose.yaw;
        }
        return aprilTagYaw;
    }

    public double getAdjustmentSpeed(AprilTagDetection detectedId) {
        if (detectedId == null) {
            return -1000;
        }
        if (detectedId.metadata != null) {
            aprilTagYDistanceAdjusting = detectedId.ftcPose.y;
        }

        if (aprilTagYDistanceAdjusting <= 240.0) {
            RotateSpeed = 1;
        } else if (aprilTagYDistanceAdjusting <= 400) {
            RotateSpeed = 0.4;
        } else {
            RotateSpeed = 0.2;
        }

        return RotateSpeed;
    }



    public double calculateVelocity(AprilTagDetection detectedId) { // uses linear interpolation to estimate a flywheel velocity based on vertical distance from april tag
        if (detectedId == null) {return -1;}
        if (detectedId.metadata != null) {
            aprilTagYDistanceVelocity = detectedId.ftcPose.y;
            if (aprilTagYDistanceVelocity <= 300.0) {
                return 1.2615 * aprilTagYDistanceVelocity + 970.38 - 100; //100 is the offset to prevent overshooting from close distance ;
            } else {
                return 1.2615 * aprilTagYDistanceVelocity + 970.38;
            }
        } else {
            return -1;
        }
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
