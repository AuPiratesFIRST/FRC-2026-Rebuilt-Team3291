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

/**
 * Turret Subsystem - Provides auto-aiming by calculating desired robot heading.
 * 
 * IMPORTANT: This is NOT a physical rotating turret! Instead, it's a "virtual turret"
 * that calculates what direction the entire robot should face to aim at the hub.
 * 
 * How it works:
 * 1. Continuously calculates angle from robot to hub based on field position
 * 2. When enabled, runs a PID controller to determine rotation speed (omega)
 * 3. Drive commands read this omega value and rotate the robot accordingly
 * 
 * This design allows the driver to control forward/backward movement while
 * auto-aim handles rotation - called "heading lock" or "turret mode".
 */
public class TurretSubsystem extends SubsystemBase {

        // Distance from robot center to shooter (meters)
        // This accounts for the fact that the shooter isn't at the robot's center
        private static final Translation2d SHOOTER_OFFSET = new Translation2d(0.35, 0.0);

        // Maximum rotation speed when auto-aiming (rad/sec)
        // Limited to prevent spinning too fast and losing control
        private static final double MAX_OMEGA_RAD_PER_SEC = 3.0;

        // Subsystem dependencies - we need these to calculate aiming
        private final VisionSubsystem vision;      // For distance measurements (future use)
        private final Drive drive;                  // For current robot pose
        private final ShooterSubsystem shooter;     // For visualization
        private final HoodSubsystem hood;           // For visualization
        private final TurretVisualizer visualizer;  // 3D visualization in AdvantageScope

        // ========== STATE VARIABLES ==========
        // These track whether we're in auto-aim mode or manual control
        private boolean hubTrackingEnabled = false;  // Is auto-aim active?
        private double manualOmega = 0.0;            // Manual rotation from driver

        // ========== CALCULATED VALUES ==========
        // Updated every loop in periodic()
        private Rotation2d desiredFieldHeading = new Rotation2d();  // Which way should we face?
        private double distanceToHubMeters = 0.0;                   // How far to target?

        // ========== PID CONTROLLER ==========
        // Calculates rotation speed needed to reach desired heading
        // Gains: kP=6.0 (aggressive), kI=0.0 (no integral), kD=0.25 (light damping)
        private final PIDController headingPID = new PIDController(6.0, 0.0, 0.25);

        /**
         * Creates a new TurretSubsystem.
         * Sets up PID controller and visualization.
         * 
         * @param vision Vision subsystem (for future distance-based aiming)
         * @param drive Drive subsystem (to read robot pose)
         * @param shooter Shooter subsystem (for visualization)
         * @param hood Hood subsystem (for visualization)
         */
        public TurretSubsystem(
                        VisionSubsystem vision,
                        Drive drive,
                        ShooterSubsystem shooter,
                        HoodSubsystem hood) {

                this.vision = vision;
                this.drive = drive;
                this.shooter = shooter;
                this.hood = hood;

                // Enable continuous input for heading PID (wraps around at ±π)
                // This ensures -179° and +179° are treated as close together
                headingPID.enableContinuousInput(-Math.PI, Math.PI);

                // Create visualizer for 3D view in AdvantageScope
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
