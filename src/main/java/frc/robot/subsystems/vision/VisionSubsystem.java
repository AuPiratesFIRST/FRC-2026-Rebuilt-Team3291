package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.simulation.*;
import org.photonvision.targeting.*;

public class VisionSubsystem extends SubsystemBase {

    // ============================================================
    // CAMERAS
    // ============================================================
    private final PhotonCamera frontCamera;
    private final PhotonCamera shooterCamera;

    // ============================================================
    // FIELD + POSE ESTIMATION
    // ============================================================
    private final AprilTagFieldLayout fieldLayout;

    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator shooterPoseEstimator;

    private final StructPublisher<Pose2d> visionPosePub;

    // ============================================================
    // SIMULATION
    // ============================================================
    private VisionSystemSim visionSim;
    private PhotonCameraSim frontCameraSim;
    private PhotonCameraSim shooterCameraSim;

    // ============================================================
    // CONSTRUCTOR
    // ============================================================

    public VisionSubsystem() {

        // Connect to PhotonVision cameras by name
        frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
        shooterCamera = new PhotonCamera(SHOOTER_CAMERA_NAME);

        // Load Field Layout
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // NEW PHOTONVISION API: Just FieldLayout + Transform3d in constructor
        frontPoseEstimator = new PhotonPoseEstimator(fieldLayout, ROBOT_TO_FRONT_CAMERA);
        shooterPoseEstimator = new PhotonPoseEstimator(fieldLayout, ROBOT_TO_SHOOTER_CAMERA);

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

        frontCameraSim = new PhotonCameraSim(frontCamera, props);
        shooterCameraSim = new PhotonCameraSim(shooterCamera, props);

        visionSim.addCamera(frontCameraSim, ROBOT_TO_FRONT_CAMERA);
        visionSim.addCamera(shooterCameraSim, ROBOT_TO_SHOOTER_CAMERA);
    }

    public void updateSimPose(Pose2d robotPose) {
        if (visionSim != null) {
            visionSim.update(robotPose);
        }
    }

    // ============================================================
    // GLOBAL POSE ESTIMATION (DUAL CAMERA - NEW API)
    // ============================================================

    /**
     * Loops through unread frames from BOTH cameras to generate a list of field
     * poses.
     */
    public List<EstimatedRobotPose> getEstimatedGlobalPoses() {
        List<EstimatedRobotPose> estimates = new ArrayList<>();

        // Process all unseen frames from the Front Camera
        for (var result : frontCamera.getAllUnreadResults()) {
            // Primary strategy: Multi-tag PNP
            Optional<EstimatedRobotPose> pose = frontPoseEstimator.estimateCoprocMultiTagPose(result);

            // Fallback strategy: Lowest Ambiguity (Single Tag)
            if (pose.isEmpty()) {
                pose = frontPoseEstimator.estimateLowestAmbiguityPose(result);
            }
            pose.ifPresent(estimates::add);
        }

        // Process all unseen frames from the Shooter Camera
        for (var result : shooterCamera.getAllUnreadResults()) {
            // Primary strategy: Multi-tag PNP
            Optional<EstimatedRobotPose> pose = shooterPoseEstimator.estimateCoprocMultiTagPose(result);

            // Fallback strategy: Lowest Ambiguity (Single Tag)
            if (pose.isEmpty()) {
                pose = shooterPoseEstimator.estimateLowestAmbiguityPose(result);
            }
            pose.ifPresent(estimates::add);
        }

        return estimates;
    }

    // ============================================================
    // DISTANCE TO TAG (PLANAR) - Uses Shooter Camera
    // ============================================================

    public Optional<Double> getDistanceToTagMeters(int[] validTags) {
        PhotonPipelineResult result = shooterCamera.getLatestResult();
        if (!result.hasTargets())
            return Optional.empty();

        for (PhotonTrackedTarget target : result.getTargets()) {
            for (int id : validTags) {
                if (target.getFiducialId() == id) {
                    return Optional.of(target.getBestCameraToTarget().getTranslation().getNorm());
                }
            }
        }
        return Optional.empty();
    }

    // ============================================================
    // TAG ROTATION RELATIVE TO CAMERA
    // ============================================================
    public Optional<Double> getTagRotationRelativeRad(int[] validTags) {
        PhotonPipelineResult result = shooterCamera.getLatestResult();
        if (!result.hasTargets())
            return Optional.empty();

        for (PhotonTrackedTarget target : result.getTargets()) {
            for (int id : validTags) {
                if (target.getFiducialId() == id) {
                    return Optional.of(target.getBestCameraToTarget().getRotation().getZ());
                }
            }
        }
        return Optional.empty();
    }

    // ============================================================
    // LATERAL OFFSET TO TAG
    // ============================================================
    public Optional<Double> getTargetLateralOffsetMeters(int[] validTags) {
        PhotonPipelineResult result = shooterCamera.getLatestResult();
        if (!result.hasTargets())
            return Optional.empty();

        for (PhotonTrackedTarget target : result.getTargets()) {
            for (int id : validTags) {
                if (target.getFiducialId() == id) {
                    return Optional.of(target.getBestCameraToTarget().getY());
                }
            }
        }
        return Optional.empty();
    }

    // ============================================================
    // YAW TO TARGET
    // ============================================================

    public Optional<Double> getTargetYawRad(int[] validTags) {
        PhotonPipelineResult result = shooterCamera.getLatestResult();
        if (!result.hasTargets())
            return Optional.empty();

        for (PhotonTrackedTarget target : result.getTargets()) {
            for (int id : validTags) {
                if (target.getFiducialId() == id) {
                    return Optional.of(Units.degreesToRadians(target.getYaw()));
                }
            }
        }
        return Optional.empty();
    }

    // ============================================================
    // ROBOT POSE IN TARGET SPACE
    // ============================================================
    public Optional<Pose2d> getRobotPoseInTargetSpace(int[] validTags) {
        var result = shooterCamera.getLatestResult();
        if (!result.hasTargets())
            return Optional.empty();

        for (var target : result.getTargets()) {
            for (int id : validTags) {
                if (target.getFiducialId() == id) {
                    Transform3d camToTarget = target.getBestCameraToTarget();
                    Transform3d targetToCam = camToTarget.inverse();

                    return Optional.of(new Pose2d(
                            targetToCam.getX(),
                            targetToCam.getY(),
                            new Rotation2d(targetToCam.getRotation().getZ())));
                }
            }
        }
        return Optional.empty();
    }

    public PhotonPipelineResult getLatestResult() {
        return shooterCamera.getLatestResult();
    }

    public PhotonPipelineResult getFrontCameraResult() {
        return frontCamera.getLatestResult();
    }

    // ============================================================
    // PERIODIC LOGGING
    // ============================================================

    @Override
    public void periodic() {
        for (EstimatedRobotPose est : getEstimatedGlobalPoses()) {
            visionPosePub.set(est.estimatedPose.toPose2d());
            break; // Just grab one to display an icon on AdvantageScope/Glass
        }
    }
}