package frc.robot.subsystems.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.Turret.TurretVisualizer;
import frc.robot.util.FuelSim;
import edu.wpi.first.wpilibj2.command.Command;

public class TurretSubsystem extends SubsystemBase {

        // ------------------------------------------------
        // CONSTANTS
        // ------------------------------------------------

        private static final Translation2d SHOOTER_OFFSET = new Translation2d(0.35,
                        0.0);

        // ========== FUEL STORAGE ==========
        private int fuelStored = 5;
        public static final int FUEL_CAPACITY = 200; // Example capacity for fuel

        // ------------------------------------------------
        // DEPENDENCIES
        // ------------------------------------------------

        private final VisionSubsystem vision;
        private final SwerveSubsystem swerve;
        private final ShooterSubsystem shooter; // For visualization
        private final HoodSubsystem hood; // For visualization
        private final FuelSim fuelSim;
        // ADDED
        private final TurretVisualizer visualizer;

        // ------------------------------------------------
        // STATE
        // ------------------------------------------------

        private boolean hubTrackingEnabled = false;
        private double manualOmega = 0.0;

        private Rotation2d desiredFieldHeading = new Rotation2d();
        private double distanceToHubMeters = 0.0;

        // ------------------------------------------------
        // CONTROLLERS
        // ------------------------------------------------

        private final PIDController headingPID = new PIDController(2.0, 0.0, 0.25);

        // ------------------------------------------------
        // CONSTRUCTOR
        // ------------------------------------------------

        public TurretSubsystem(
                        VisionSubsystem vision,
                        SwerveSubsystem swerve,
                        ShooterSubsystem shooter,
                        HoodSubsystem hood,
                        FuelSim fuelSim) {

                this.vision = vision;
                this.swerve = swerve;
                this.shooter = shooter;
                this.hood = hood;
                this.fuelSim = fuelSim; // Assign the passed FuelSi

                headingPID.enableContinuousInput(-Math.PI, Math.PI);

                // ADDED
                visualizer = new TurretVisualizer(
                                fuelSim,
                                () -> new Pose3d(

                                                swerve.getPose().getTranslation().getX(),
                                                swerve.getPose().getTranslation().getY(),
                                                0.0,
                                                new Rotation3d(
                                                                0,
                                                                0,
                                                                swerve.getPose().getRotation().getRadians())),
                                swerve::getFieldVelocity,
                                () -> DriverStation.getAlliance()
                                                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue);
        }

        // ------------------------------------------------
        // PERIODIC
        // ------------------------------------------------

        @Override
        public void periodic() {
                updateTargeting();
                logToAdvantageScope();

                // FIX: correct shooter velocity getter
                visualizer.update(
                                shooter.getTargetRPM(),
                                hood.getTargetAngle());
        }

        // ------------------------------------------------
        // TARGETING MATH
        // ------------------------------------------------

        private void updateTargeting() {

                Translation3d hub = DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                                ? Constants.FieldConstants.HUB_BLUE
                                                : Constants.FieldConstants.HUB_RED;

                Pose2d robotPose = swerve.getPose();
                Rotation2d robotYaw = robotPose.getRotation();

                Translation2d shooterFieldPos = robotPose.getTranslation()
                                .plus(SHOOTER_OFFSET.rotateBy(robotYaw));

                Translation2d toHub = hub.toTranslation2d().minus(shooterFieldPos);

                desiredFieldHeading = new Rotation2d(
                                Math.atan2(toHub.getY(), toHub.getX()));

                distanceToHubMeters = toHub.getNorm();
        }

        // ------------------------------------------------
        // CONTROL API
        // ------------------------------------------------

        public void enableHubTracking() {
                hubTrackingEnabled = true;
        }

        public void disableHubTracking() {
                hubTrackingEnabled = false;
                manualOmega = 0.0;
        }

        public void manualRotate(double omega) {
                hubTrackingEnabled = false;
                manualOmega = omega;

                if (Math.abs(omega) > 0.05) {
                        hubTrackingEnabled = false;
                }
        }

        public double getDesiredRobotOmega() {
                if (hubTrackingEnabled) {
                        return headingPID.calculate(
                                        swerve.getPose().getRotation().getRadians(),
                                        desiredFieldHeading.getRadians());
                }
                return manualOmega;
        }

        public Rotation2d getDesiredRobotHeading() {
                return desiredFieldHeading;
        }

        public double getDistanceToHubMeters() {
                return distanceToHubMeters;
        }

        // ------------------------------------------------
        // ACCESSORS
        // ------------------------------------------------
        public boolean isHubTrackingEnabled() {
                return hubTrackingEnabled;
        }

        // Fuel storage and shooting methods
        public void intakeFuel() {
                if (fuelStored < FUEL_CAPACITY) {
                        fuelStored++;
                        System.out.println("Fuel intaken! Total: " + fuelStored); // For debugging
                }
        }

        public boolean canShoot() {
                return fuelStored > 0;
        }

        public int getFuelStored() {
                return fuelStored;
        }

        public Command shootCommand() {
                return run(this::shoot);
        }

        public void resetFuelStored() {
                fuelStored = 0;
                System.out.println("Fuel stored reset to 0.");
        }

        public void shoot() {
                if (!edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
                        return;
                }

                if (canShoot()) { // Check if we have fuel
                        fuelStored--; // Decrement fuel count
                        System.out.println("Shooting fuel! Remaining: " + fuelStored); // For debugging
                        visualizer.fire(
                                        shooter.getTargetRPM(),
                                        hood.getTargetAngle());
                } else {
                        System.out.println("Cannot shoot: No fuel stored."); // For debugging
                }
        }

        // ------------------------------------------------
        // LOGGING (ADVANTAGESCOPE)
        // ------------------------------------------------

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
                                swerve.getPose().getRotation().getDegrees());

                SmartDashboard.putNumber(
                                "Targeting/HeadingErrorDeg",
                                desiredFieldHeading
                                                .minus(swerve.getPose().getRotation())
                                                .getDegrees());
        }
}
