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

public class VisionSubsystem extends SubsystemBase {

    // ============================================================
    // CAMERA (SINGLE, MULTI-PURPOSE)
    // ============================================================

    private final PhotonCamera camera;

    // ============================================================
    // FIELD + POSE
    // ============================================================

    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator poseEstimator;
    private final StructPublisher<Pose2d> visionPosePub;

    // ============================================================
    // SIMULATION
    // ============================================================

    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;

    // ============================================================
    // CONSTRUCTOR
    // ============================================================

    public VisionSubsystem() {

        camera = new PhotonCamera(SHOOTER_CAMERA_NAME);

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
