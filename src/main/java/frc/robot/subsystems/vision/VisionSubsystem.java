package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.*;
import org.photonvision.simulation.*;
import org.photonvision.targeting.*;

public class VisionSubsystem extends SubsystemBase {

    // ---------- CAMERA ----------
    private final PhotonCamera shooterCamera;

    // ---------- FIELD ----------
    private final AprilTagFieldLayout fieldLayout;

    // ---------- SIM ----------
    private VisionSystemSim visionSim;
    private PhotonCameraSim shooterCamSim;

    public VisionSubsystem() {

        shooterCamera = new PhotonCamera(SHOOTER_CAMERA_NAME);

        fieldLayout = AprilTagFieldLayout.loadField(
                AprilTagFields.k2026RebuiltWelded);

        if (RobotBase.isSimulation()) {
            setupSimulation();
        }
    }

    // ---------------- SIM ----------------
    private void setupSimulation() {

        visionSim = new VisionSystemSim("Vision");

        visionSim.addAprilTags(fieldLayout);

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        props.setFPS(30);
        props.setAvgLatencyMs(35);

        shooterCamSim = new PhotonCameraSim(shooterCamera, props);

        visionSim.addCamera(
                shooterCamSim,
                ROBOT_TO_SHOOTER_CAMERA);
    }

    public void updateSimPose(Pose2d robotPose) {
        if (visionSim != null) {
            visionSim.update(robotPose);
        }
    }

    // ---------------- DISTANCE (SHOOTER CAMERA) ----------------
    public Optional<Double> getDistanceToTagMeters(int[] validTags) {

        PhotonPipelineResult result = shooterCamera.getLatestResult();

        if (!result.hasTargets())
            return Optional.empty();

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

    // ---------------- SHOOTER YAW ONLY ----------------
    public Optional<Double> getShooterTargetYawRad(int[] validTags) {

        PhotonPipelineResult result = shooterCamera.getLatestResult();

        if (!result.hasTargets())
            return Optional.empty();

        for (PhotonTrackedTarget target : result.getTargets()) {
            for (int id : validTags) {
                if (target.getFiducialId() == id) {

                    // Photon yaw is degrees, CCW positive
                    return Optional.of(
                            Units.degreesToRadians(
                                    target.getYaw()));
                }
            }
        }
        return Optional.empty();
    }

    @Override
    public void periodic() {
        // Shooter-only vision — nothing to publish
    }
}
