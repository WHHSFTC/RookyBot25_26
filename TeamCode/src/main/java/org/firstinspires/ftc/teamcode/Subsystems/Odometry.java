package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

// Imports for Pinpoint Driver
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// We need Pose3D and its components to accept the pose from the Limelight
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


/**
 * Odometry subsystem class for handling the goBILDA Pinpoint.
 * This class wraps the driver, manages initialization, and provides
 * clean methods for getting the robot's pose and handling relocalization.
 */
public class Odometry {

    // The core Pinpoint hardware driver
    private final GoBildaPinpointDriver odo;

    // Store the current pose in the Pinpoint's native units (MM and Radians)
    private Pose2D currentPose;

    /**
     * Constructor: Initializes the Pinpoint hardware.
     * @param hardwareMap The robot's hardware map.
     */
    public Odometry(HardwareMap hardwareMap) {
        // "odo" must match the name in your robot's configuration file
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // The Pose2D constructor requires 5 arguments (units + values)
        // We initialize to 0,0,0 in the Pinpoint's native units (MM and RADIANS)
        currentPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    }

    /**
     * Configures the Pinpoint driver with your robot's specific settings
     * and sets its starting position on the field.
     *
     * MUST BE CALLED IN YOUR OPMODE'S init() METHOD.
     *
     * @param startingFtcPose The robot's known starting Pose2D in FTC Field Coordinates (X=strafe, Y=fwd, Units=INCHES or MM)
     */
    public void init(Pose2D startingFtcPose) {
        // --- CONFIGURE YOUR ODOMETRY PODS HERE ---
        // (These are examples, use your robot's real measurements in MM)
        double xPodOffset_mm = -84.0;
        double yPodOffset_mm = -168.0;

        odo.setOffsets(xPodOffset_mm, yPodOffset_mm, DistanceUnit.MM);

        // Set which pod type you are using
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // or odo.setEncoderResolution(ticks_per_mm, DistanceUnit.MM);

        // Set the counting direction of your encoders
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD, // X (fwd) pod
                GoBildaPinpointDriver.EncoderDirection.FORWARD  // Y (strafe) pod
        );

        // This is the most important part:
        // Relocalize to the known starting pose.
        relocalize(startingFtcPose);
    }

    /**
     * --- THIS METHOD MUST BE CALLED IN YOUR OPMODE'S loop() ---
     * Polls the Pinpoint for the latest pose and updates the internal state.
     */
    public void update() {
        odo.update();
        currentPose = odo.getPosition(); // odo.getPosition() returns a Pose2D
    }

    /**
     * Gets the robot's current position in Pinpoint's native coordinate system.
     * @return Pose2D (X=Forward, Y=Strafe, Heading=CCW) in MM and Radians
     */
    public Pose2D getPose() {
        return currentPose;
    }

    /**
     * Gets the robot's current position in the standard FTC Field coordinate system.
     * @return Pose2D (X=Strafe, Y=Forward, Heading=CCW) in MM and Radians
     */
    public Pose2D getFtcPose() {
        double pinpointX_mm = currentPose.getX(DistanceUnit.MM);
        double pinpointY_mm = currentPose.getY(DistanceUnit.MM);
        double pinpointHeading_rad = currentPose.getHeading(AngleUnit.RADIANS);

        // Convert Pinpoint's (X=fwd, Y=strafe) to FTC's (X=strafe, Y=fwd)
        // Pinpoint's +Y is Left, FTC's +X is Right. So FTC X = -Pinpoint Y
        double ftcX_mm = -pinpointY_mm;
        // Pinpoint's +X is Forward, FTC's +Y is Forward. So FTC Y = Pinpoint X
        double ftcY_mm = pinpointX_mm;

        return new Pose2D(DistanceUnit.MM, ftcX_mm, ftcY_mm, AngleUnit.RADIANS, pinpointHeading_rad);
    }


    /**
     * Resets the Pinpoint's internal pose to a new, absolute field pose.
     * This version accepts a 2D pose, like from an Auto's starting position.
     * This method handles the coordinate system conversion.
     *
     * @param ftcPose The absolute pose in standard FTC coordinates (X=Strafe, Y=Fwd)
     */
    public void relocalize(Pose2D ftcPose) {
        // --- COORDINATE SYSTEM CONVERSION ---
        // Convert FTC coordinates (X=Strafe, Y=Fwd) to Pinpoint coordinates (X=Fwd, Y=Strafe)

        // Pinpoint X (fwd) = FTC Y (fwd)
        double pinpointX_mm = ftcPose.getY(DistanceUnit.MM);

        // Pinpoint Y (strafe) = FTC -X (strafe)
        // (Pinpoint's +Y is Left, FTC's +X is Right. So Pinpoint Y = -FTC X)
        double pinpointY_mm = -ftcPose.getX(DistanceUnit.MM);

        // Heading is the same (CCW positive)
        double pinpointHeading_rad = ftcPose.getHeading(AngleUnit.RADIANS);

        // Create a new Pose2D in Pinpoint's native units (MM and RADIANS)
        // The Pinpoint driver's setPosition() method expects this format.
        Pose2D pinpointPose = new Pose2D(DistanceUnit.MM, pinpointX_mm, pinpointY_mm, AngleUnit.RADIANS, pinpointHeading_rad);

        // Set the Pinpoint's internal position to this new absolute pose
        odo.setPosition(pinpointPose);

        // Update our internal pose variable as well
        currentPose = pinpointPose;
    }

    /**
     * Overloaded relocalize method to accept a 3D pose from the Limelight.
     *
     * @param limelightPose The absolute Pose3D from the Limelight (Botpose)
     * (Assumes FTC coordinates in METERS)
     */
    public void relocalize(Pose3D limelightPose) {
        if (limelightPose == null) return;

        // Limelight's getBotpose() returns a Pose3D in METERS
        // in the FTC coordinate system (X=Strafe, Y=Fwd)

        // The Pose3D object's position is a Position object.
        // We can access x (Strafe) and y (Forward) directly.
        double ftcX_meters = limelightPose.getPosition().x;
        double ftcY_meters = limelightPose.getPosition().y;

        // It uses a YawPitchRollAngles object for rotation.
        // We can get the yaw (heading) from it.
        double ftcHeading_rad = limelightPose.getOrientation().getYaw(AngleUnit.RADIANS);

        // Create a 2D pose from the Limelight data
        Pose2D ftcPose2D = new Pose2D(DistanceUnit.METER, ftcX_meters, ftcY_meters, AngleUnit.RADIANS, ftcHeading_rad);

        // Call our other relocalize method. It handles the coordinate swap
        // and unit conversion (Pose2D objects handle this automatically).
        relocalize(ftcPose2D);
    }
}
