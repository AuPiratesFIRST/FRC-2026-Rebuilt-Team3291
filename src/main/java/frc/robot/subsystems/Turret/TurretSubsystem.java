package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.TankDrive.Drive;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.HoodSubsystem;

public class TurretSubsystem extends SubsystemBase {

        private static final Translation2d SHOOTER_OFFSET = new Translation2d(0.35, 0.0);

        private static final double MAX_OMEGA_RAD_PER_SEC = 3.0;

        private final VisionSubsystem vision;
        private final Drive drive;
        private final ShooterSubsystem shooter;
        private final HoodSubsystem hood;
        private final TurretVisualizer visualizer;

        private boolean hubTrackingEnabled = false;
        private double manualOmega = 0.0;

        private Rotation2d desiredFieldHeading = new Rotation2d();
        private double distanceToHubMeters = 0.0;

        private final PIDController headingPID = new PIDController(6.0, 0.0, 0.25);

        public TurretSubsystem(
                        VisionSubsystem vision,
                        Drive drive,
                        ShooterSubsystem shooter,
                        HoodSubsystem hood) {

                this.vision = vision;
                this.drive = drive;
                this.shooter = shooter;
                this.hood = hood;

                headingPID.enableContinuousInput(-Math.PI, Math.PI);

                visualizer = new TurretVisualizer(
                                () -> new Pose3d(drive.getPose()),
                                drive::getChassisSpeeds,
                                () -> DriverStation.getAlliance()
                                                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue);
        }

        @Override
        public void periodic() {
                updateTargeting();
                logToAdvantageScope();

                visualizer.update(
                                shooter.getTargetRPM(),
                                hood.getTargetAngle());
        }

        private void updateTargeting() {
                Translation3d hub = DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                                ? Constants.FieldConstants.HUB_BLUE
                                                : Constants.FieldConstants.HUB_RED;

                Pose2d robotPose = drive.getPose();
                Rotation2d robotYaw = robotPose.getRotation();

                Translation2d shooterFieldPos = robotPose.getTranslation()
                                .plus(SHOOTER_OFFSET.rotateBy(robotYaw));

                Translation2d toHub = hub.toTranslation2d().minus(shooterFieldPos);

                desiredFieldHeading = new Rotation2d(Math.atan2(
                                toHub.getY(), toHub.getX()));

                distanceToHubMeters = toHub.getNorm();
        }

        // ---------------- CONTROL API ----------------

        public void enableHubTracking() {
                headingPID.reset();
                hubTrackingEnabled = true;
        }

        public boolean isHubTrackingEnabled() {
                return hubTrackingEnabled;
        }

        public void disableHubTracking() {
                hubTrackingEnabled = false;
                manualOmega = 0.0;
        }

        public void manualRotate(double omega) {
                manualOmega = omega;
                if (Math.abs(omega) > 0.05) {
                        hubTrackingEnabled = false;
                }
        }

        public double getDesiredRobotOmega() {
                double omega;

                if (hubTrackingEnabled) {
                        omega = headingPID.calculate(
                                        drive.getPose().getRotation().getRadians(),
                                        desiredFieldHeading.getRadians());
                } else {
                        omega = manualOmega;
                }

                return MathUtil.clamp(
                                omega,
                                -MAX_OMEGA_RAD_PER_SEC,
                                MAX_OMEGA_RAD_PER_SEC);
        }

        public double getDistanceToHubMeters() {
                return distanceToHubMeters;
        }

        private void logToAdvantageScope() {
                SmartDashboard.putBoolean(
                                "Turret/HubTrackingEnabled",
                                hubTrackingEnabled);

                SmartDashboard.putNumber(
                                "Targeting/DistanceMeters",
                                distanceToHubMeters);

                SmartDashboard.putNumber(
                                "Targeting/DesiredHeadingDeg",
                                desiredFieldHeading.getDegrees());

                SmartDashboard.putNumber(
                                "Targeting/RobotYawDeg",
                                drive.getPose().getRotation().getDegrees());
        }
}
