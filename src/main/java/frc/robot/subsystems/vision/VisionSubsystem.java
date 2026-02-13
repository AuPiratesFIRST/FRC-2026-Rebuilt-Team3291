package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.*;
import org.photonvision.simulation.*;
import org.photonvision.targeting.*;

/**
 * Vision Subsystem - Detects AprilTags using PhotonVision for pose estimation and targeting.
 * 
 * This subsystem manages the robot's camera and processes AprilTag detections to:
 * 1. Estimate the robot's global position on the field (pose estimation)
 * 2. Measure distance and angle to specific tags for shooter aiming
 * 3. Provide realistic simulation data for testing
 * 
 * PhotonVision runs on a coprocessor (like a Raspberry Pi) and sends results
 * to the roboRIO via NetworkTables. This subsystem reads those results.
 */
public class VisionSubsystem extends SubsystemBase {

    // ============================================================
    // CAMERA (SINGLE, MULTI-PURPOSE)
    // ============================================================
    // PhotonCamera connects to PhotonVision instance running on coprocessor
    // Camera name must match the name configured in the PhotonVision web interface
    private final PhotonCamera camera;

    // ============================================================
    // FIELD + POSE ESTIMATION
    // ============================================================
    // AprilTag field layout contains the 3D positions of all tags on the field
    // This is loaded from WPILib's built-in field layouts (updated yearly)
    private final AprilTagFieldLayout fieldLayout;
    
    // PhotonPoseEstimator uses detected tags + field layout to estimate robot pose
    // Uses MULTI_TAG_PNP_ON_COPROCESSOR strategy for best accuracy with multiple tags
    private final PhotonPoseEstimator poseEstimator;
    
    // NetworkTables publisher to share vision pose with other programs (AdvantageScope, etc.)
    private final StructPublisher<Pose2d> visionPosePub;

    // ============================================================
    // SIMULATION
    // ============================================================
    // These are only used in simulation to provide realistic camera behavior
    // null on real robot
    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;

    // ============================================================
    // CONSTRUCTOR
    // ============================================================

    /**
     * Creates a new VisionSubsystem.
     * Initializes camera connection, loads field layout, and sets up simulation if needed.
     */
    public VisionSubsystem() {

        // Connect to PhotonVision camera by name (must match PhotonVision config)
        camera = new PhotonCamera(SHOOTER_CAMERA_NAME);

        // Load the official 2026 Rebulit field AprilTag layout
        // This contains the 3D position of every AprilTag on the field
        fieldLayout = AprilTagFieldLayout.loadField(
                AprilTagFields.k2026RebuiltWelded);

        poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                ROBOT_TO_SHOOTER_CAMERA);

        visionPosePub = NetworkTableInstance.getDefault()
                .getStructTopic("Vision/EstimatedPose", Pose2d.struct)
                .publish();

        if (RobotBase.isSimulation()) {
            setupSimulation();
        }
    }

    // ============================================================
    // SIM SETUP
    // ============================================================

    private void setupSimulation() {

        visionSim = new VisionSystemSim("Vision");

        visionSim.addAprilTags(fieldLayout);

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        props.setFPS(30);
        props.setAvgLatencyMs(35);
        props.setCalibError(0.25, 0.25);

        cameraSim = new PhotonCameraSim(camera, props);

        visionSim.addCamera(
                cameraSim,
                ROBOT_TO_SHOOTER_CAMERA);
    }

    /** MUST be called from DriveSubsystem.simulationPeriodic() */
    public void updateSimPose(Pose2d robotPose) {
        if (visionSim != null) {
            visionSim.update(robotPose);
        }
    }

    // ============================================================
    // GLOBAL POSE ESTIMATION
    // ============================================================

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {

        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            return Optional.empty();
        }

        return poseEstimator.update(result);
    }

    // ============================================================
    // DISTANCE TO TAG (PLANAR)
    // ============================================================

    public Optional<Double> getDistanceToTagMeters(int[] validTags) {

        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            return Optional.empty();
        }

        for (PhotonTrackedTarget target : result.getTargets()) {
            for (int id : validTags) {
                if (target.getFiducialId() == id) {

                    Transform3d camToTarget = target.getBestCameraToTarget();

                    return Optional.of(
                            camToTarget.getTranslation().getNorm());
                }
            }
        }
        return Optional.empty();
    }

    // ============================================================
    // YAW TO TARGET (FOR AIMING)
    // ============================================================

    public Optional<Double> getTargetYawRad(int[] validTags) {

        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            return Optional.empty();
        }

        for (PhotonTrackedTarget target : result.getTargets()) {
            for (int id : validTags) {
                if (target.getFiducialId() == id) {

                    // Photon yaw is degrees, CCW+
                    return Optional.of(
                            Units.degreesToRadians(target.getYaw()));
                }
            }
        }
        return Optional.empty();
    }

    // ============================================================
    // PERIODIC LOGGING
    // ============================================================

    @Override
    public void periodic() {

        getEstimatedGlobalPose().ifPresent(pose -> visionPosePub.set(
                pose.estimatedPose.toPose2d()));
    }
}
