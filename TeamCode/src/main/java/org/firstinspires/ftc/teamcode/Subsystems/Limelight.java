package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes; // Import for AprilTag (Fiducial) results
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.json.JSONException; // Required for LLResult constructor

import java.util.List;

/**
 * Limelight subsystem class for handling all Limelight-related logic.
 * This class abstracts away the raw API calls and provides simple methods
 * for getting AprilTag data and robot pose.
 */
public class Limelight {

    // The core Limelight hardware object
    private final Limelight3A limelight;

    // A variable to store the most recent valid result
    // Initialize to null. We will get a valid result from getLatestResult() in the update() method.
    private LLResult latestResult = null;

    // --- Alliance and Tag Constants ---

    // Define an enum for alliance color to make code more readable
    public enum Alliance {
        RED,
        BLUE
    }

    // Define an enum for the Obelisk Motifs (from Game Manual Section 9.6)
    public enum Motif {
        GPP,    // ID 21
        PGP,    // ID 22
        PPG,    // ID 23
        UNKNOWN // No Obelisk tag visible
    }

    // Store the current alliance
    private Alliance currentAlliance = Alliance.BLUE; // Default to BLUE

    // AprilTag IDs for the DECODE Goals (from Game Manual Section 9.10)
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID = 24;

    // AprilTag IDs for the Obelisk (from Game Manual Section 9.10)
    public static final int OBELISK_TAG_GPP = 21;
    public static final int OBELISK_TAG_PGP = 22;
    public static final int OBELISK_TAG_PPG = 23;


    // --- Pipeline Constants ---
    // These should match the pipeline setup in your Limelight web interface
    public static final int TARGETING_PIPELINE_INDEX = 0; // Pipeline for GOAL AprilTags
    public static final int OBELISK_PIPELINE_INDEX = 1;   // Pipeline for OBELISK AprilTags

    // Constants for hardware map and configuration
    private static final String HARDWARE_MAP_NAME = "limelight";
    private static final int POLLING_RATE_HZ = 100;

    /**
     * Constructor: Initializes the Limelight hardware.
     * @param hardwareMap The robot's hardware map, passed from the OpMode.
     */
    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, HARDWARE_MAP_NAME);
        limelight.setPollRateHz(POLLING_RATE_HZ);
        limelight.start();

        // We no longer create a new LLResult() here.
        // latestResult is already null, which is the correct starting state.

        setPipeline(TARGETING_PIPELINE_INDEX); // Default to targeting pipeline on startup
    }

    /**
     * --- THIS METHOD MUST BE CALLED IN THE OPMODE'S loop() ---
     * Polls the Limelight for the latest result and updates the internal state.
     * This keeps the data (like latestResult) fresh.
     */
    public void update() {
        LLResult result = limelight.getLatestResult();
        // Only update our internal state if the new result is valid
        if (result != null && result.isValid()) {
            this.latestResult = result;
        }
    }

    /**
     * Stops the Limelight stream. Call this at the end of an OpMode
     * in the stop() method to save resources.
     */
    public void stop() {
        limelight.stop();
    }

    // --- CONTROL METHODS ---

    /**
     * Sets the current alliance using the Alliance enum.
     * This is crucial for finding the correct GOAL tag.
     * Call this in your OpMode's init() method.
     * @param alliance The current alliance (RED or BLUE).
     */
    public void setAlliance(Alliance alliance) {
        this.currentAlliance = alliance;
    }

    /**
     * Overloaded method to set the current alliance using a String.
     * This is useful for calling from OpModes where you might have the
     * alliance color as a string (e.g., "RED" or "BLUE").
     * @param allianceColor The alliance color as a string ("RED" or "BLUE"). Case-insensitive.
     */
    public void setAlliance(String allianceColor) {
        if (allianceColor != null && allianceColor.equalsIgnoreCase("RED")) {
            this.currentAlliance = Alliance.RED;
        } else {
            // Default to BLUE if the string is "BLUE", null, or anything else.
            this.currentAlliance = Alliance.BLUE;
        }
    }

    /**
     * Switches the Limelight's active pipeline.
     * @param pipelineIndex The index (0-9) of the pipeline to switch to.
     */
    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /**
     * Helper method to switch to the AprilTag targeting pipeline.
     */
    public void setTargetingPipeline() {
        setPipeline(TARGETING_PIPELINE_INDEX);
    }

    /**
     * Helper method to switch to the Obelisk detection pipeline.
     */
    public void setObeliskPipeline() {
        setPipeline(OBELISK_PIPELINE_INDEX);
    }

    /**
     * Updates the Limelight with the robot's current yaw (rotation) from the IMU.
     * This is used for MegaTag2 (MT2) pose estimation, which is more accurate.
     * @param yawInDegrees The robot's current heading from the IMU.
     */
    public void updateRobotOrientation(double yawInDegrees) {
        limelight.updateRobotOrientation(yawInDegrees);
    }

    // --- AUTONOMOUS-SPECIFIC GETTERS ---

    /**
     * Checks for a visible Obelisk tag and returns the corresponding Motif.
     * Make sure you are on the OBELISK_PIPELINE_INDEX and have called update()!
     * @return The detected Motif (GPP, PGP, PPG) or UNKNOWN if none are found.
     */
    public Motif getDetectedMotif() {
        List<LLResultTypes.FiducialResult> tags = getVisibleTags();
        if (tags == null) {
            return Motif.UNKNOWN;
        }

        for (LLResultTypes.FiducialResult tag : tags) {
            switch (tag.getFiducialId()) {
                case OBELISK_TAG_GPP:
                    return Motif.GPP;
                case OBELISK_TAG_PGP:
                    return Motif.PGP;
                case OBELISK_TAG_PPG:
                    return Motif.PPG;
            }
        }

        return Motif.UNKNOWN;
    }

    // --- ALLIANCE-SPECIFIC GOAL GETTERS ---

    /**
     * Private helper to get the FiducialResult for the correct alliance GOAL tag.
     * @return The FiducialResult for the GOAL tag, or null if not visible.
     */
    private LLResultTypes.FiducialResult getGoalTag() {
        if (!hasValidResult()) {
            return null;
        }

        // Select the correct Tag ID based on the alliance set during init()
        int targetId = (currentAlliance == Alliance.RED) ? RED_GOAL_TAG_ID : BLUE_GOAL_TAG_ID;
        return getTagById(targetId); // Use our generic tag search method
    }

    /**
     * Checks if the correct alliance GOAL tag is currently visible.
     * @return true if the GOAL tag is visible, false otherwise.
     */
    public boolean hasGoalTag() {
        return getGoalTag() != null;
    }

    /**
     * Gets the rotational error (tx) to the alliance GOAL tag.
     * This is the value you use for your P-controller for rotation.
     * @return The horizontal angle (tx) to the GOAL tag in degrees, or 0.0 if not visible.
     */
    public double getRotationErrorToGoal() {
        LLResultTypes.FiducialResult tag = getGoalTag(); // The result type for an AprilTag is FiducialResult
        if (tag == null) {
            return 0.0;
        }
        // tx (Target X) is the horizontal angle from the crosshair to the tag
        return tag.getTargetXDegrees();
    }

    /**
     * Gets the true 3D distance to the alliance GOAL tag.
     * This is the value you use to calculate your hood angle.
     * @return The distance to the GOAL tag in meters, or 0.0 if not visible.
     */
    public double getDistanceToGoal() {
        LLResultTypes.FiducialResult tag = getGoalTag();
        if (tag == null) {
            return 0.0;
        }

        Pose3D pose = tag.getRobotPoseTargetSpace();
        if (pose == null) {
            return 0.0;
        }

        // Calculate true 3D distance (hypotenuse of X and Y distances)
        // X is "forward" distance, Y is "strafe" distance
        // We get the X and Y components from the tag's 3D position
        return Math.hypot(pose.getPosition().x, pose.getPosition().y);
    }

    /**
     * Gets the strafe error to the alliance GOAL tag.
     * This can be used for an optional "auto-strafe" or just for telemetry.
     * @return The strafe distance (Y) to the GOAL tag in meters, or 0.0 if not visible.
     */
    public double getStrafeErrorToGoal() {
        LLResultTypes.FiducialResult tag = getGoalTag();
        if (tag == null) {
            return 0.0;
        }

        Pose3D pose = tag.getRobotPoseTargetSpace();
        if (pose == null) {
            return 0.0;
        }

        // Y distance from the pose is the strafe error (how far left/right we are)
        return pose.getPosition().y;
    }


    // --- GENERIC DATA GETTER METHODS ---

    /**
     * Checks if the last polled result is valid.
     * @return true if latestResult is not null and LLResult.isValid() is true.
     */
    public boolean hasValidResult() {
        return latestResult != null && latestResult.isValid();
    }

    /**
     * Gets the robot's 3D pose on the field (MegaTag1).
     * @return Pose3D object, or null if no valid pose.
     */
    public Pose3D getRobotPose() {
        if (!hasValidResult()) return null;
        return latestResult.getBotpose();
    }

    /**
     * Gets the robot's 3D pose on the field, fused with IMU data (MegaTag2).
     * Requires updateRobotOrientation() to be called.
     * @return Pose3D object, or null if no valid pose.
     */
    public Pose3D getRobotPoseMT2() {
        if (!hasValidResult()) return null;
        return latestResult.getBotpose_MT2();
    }

    /**
     * Gets a list of all currently visible AprilTags (Fiducials).
     * @return A List of FiducialResult objects, or null.
     */
    public List<LLResultTypes.FiducialResult> getVisibleTags() {
        if (!hasValidResult()) return null;
        return latestResult.getFiducialResults();
    }

    /**
     * Finds a specific AprilTag by its ID.
     * @param id The ID of the tag to find (e.g., 20).
     * @return The FiducialResult for that tag, or null if not found.
     */
    public LLResultTypes.FiducialResult getTagById(int id) {
        List<LLResultTypes.FiducialResult> tags = getVisibleTags();
        if (tags == null) {
            return null;
        }
        // Loop through all visible tags and return the one that matches the ID
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == id) {
                return tag;
            }
        }
        return null;
    }

    /**
     * Gets the horizontal angle (tx) of the primary target.
     * @return tx in degrees, or 0.0
     */
    public double getTx() {
        if (!hasValidResult()) return 0.0;
        return latestResult.getTx();
    }

    /**
     * Gets the vertical angle (ty) of the primary target.
     * @return ty in degrees, or 0.0
     */
    public double getTy() {
        if (!hasValidResult()) return 0.0;
        return latestResult.getTy();
    }

    /**
     * Gets the area (ta) of the primary target (0-100% of screen).
     * @return ta as a percentage, or 0.0
     */
    public double getTa() {
        if (!hasValidResult()) return 0.0;
        return latestResult.getTa();
    }

    /**
     * Gets the index of the currently active pipeline.
     * @return pipeline index (0-9), or -1 if no valid result.
     */
    public int getPipelineIndex() {
        if (!hasValidResult()) return -1;
        return (int) latestResult.getPipelineIndex();
    }

    /**
     * Gets the staleness of the last result (how many milliseconds old it is).
     * @return staleness in ms, or a large number if no valid result.
    Example usage in Autonomous:
    limelight.setObeliskPipeline();
    sleep(200); // Give pipeline time to switch
    limelight.update();
    Motif detectedMotif = limelight.getDetectedMotif();
    telemetry.addData("Detected Motif", detectedMotif);
     */
    public long getStaleness() {
        if (!hasValidResult()) return 9999;
        return latestResult.getStaleness();
    }
}

