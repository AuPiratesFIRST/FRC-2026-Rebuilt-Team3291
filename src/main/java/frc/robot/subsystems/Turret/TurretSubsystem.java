package frc.robot.subsystems.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterAimCalculator;
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
        // NEGATIVE offset because the shooter is on the BACK of the robot
        private static final Translation2d SHOOTER_OFFSET = new Translation2d(0.35, 0.0);

        private int fuelStored = 8;
        public static final int FUEL_CAPACITY = 200;

        // ------------------------------------------------
        // DEPENDENCIES
        // ------------------------------------------------
        private final VisionSubsystem vision;
        private final SwerveSubsystem swerve;
        private final ShooterSubsystem shooter;
        private final HoodSubsystem hood;
        private final FuelSim fuelSim;
        private final TurretVisualizer visualizer;

        // ------------------------------------------------
        // STATE
        // ------------------------------------------------
        private boolean hubTrackingEnabled = false;
        private double manualOmega = 0.0;

        private Rotation2d desiredFieldHeading = new Rotation2d();
        private double distanceToHubMeters = 0.0;

        private double calculatedRPM = 0.0;
        private double calculatedHoodAngleDeg = 0.0;

        // ------------------------------------------------
        // CONTROLLERS
        // ------------------------------------------------

        private final PIDController headingPID = new PIDController(1.74, 0.0, 0.373); // P=1.74 for smooth snapping
        private double feedforwardOmega = 0.0;
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
                this.fuelSim = fuelSim;

                headingPID.enableContinuousInput(-Math.PI, Math.PI);

                visualizer = new TurretVisualizer(
                                fuelSim,
                                () -> new Pose3d(
                                                swerve.getPose().getTranslation().getX(),
                                                swerve.getPose().getTranslation().getY(),
                                                0.0,
                                                new Rotation3d(0, 0, swerve.getPose().getRotation().getRadians())),
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
                visualizer.update(shooter.getTargetRPM(), hood.getTargetAngle());
        }

        private void updateTargeting() {
                Translation3d hub = DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                                ? Constants.FieldConstants.HUB_BLUE
                                                : Constants.FieldConstants.HUB_RED;

                Pose2d robotPose = swerve.getPose();
                ChassisSpeeds speeds = swerve.getFieldVelocity();

                // 1. Calculate the field-relative position of the shooter mechanism
                Translation2d shooterFieldPos = robotPose.getTranslation()
                                .plus(SHOOTER_OFFSET.rotateBy(robotPose.getRotation()));
                Pose2d shooterPose = new Pose2d(shooterFieldPos, robotPose.getRotation());

                // 2. Calculate Aiming Parameters
                Translation2d hub2d = hub.toTranslation2d();
                var solution = ShooterAimCalculator.solveMoving(
                                shooterPose,
                                speeds,
                                hub2d);

                // 3. Store results for logging/PID
                distanceToHubMeters = solution.distanceMeters();
                calculatedRPM = solution.rpm();

                // --- NEW KINEMATIC FEEDFORWARD MATH ---
                // Calculate vector from robot to target
                Translation2d robotToHub = hub2d.minus(robotPose.getTranslation());
                double rX = robotToHub.getX();
                double rY = robotToHub.getY();

                // Get robot's field-relative velocity
                double vX = speeds.vxMetersPerSecond;
                double vY = speeds.vyMetersPerSecond;

                // Calculate the angular velocity required to track the target while moving.
                // Equation: Omega = (rY * vX - rX * vY) / (rX^2 + rY^2)
                double distSq = (rX * rX) + (rY * rY);
                if (distSq > 0.01) { // Prevent division by zero if sitting exactly on the hub
                        feedforwardOmega = (rY * vX - rX * vY) / distSq;
                } else {
                        feedforwardOmega = 0.0;
                }
                // --------------------------------------

                // 4. THE 180-DEGREE FLIP
                desiredFieldHeading = solution.heading().plus(Rotation2d.fromDegrees(180));

                // 5. Apply to hardware
                if (hubTrackingEnabled) {
                        shooter.applyRPM(calculatedRPM);
                }
        }

        // ------------------------------------------------
        // CONTROL API
        // ------------------------------------------------

        public double getCalculatedRPM() {
                return calculatedRPM;
        }

        public double getCalculatedHoodAngleDeg() {
                return calculatedHoodAngleDeg;
        }

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
                        // The PID now only corrects for small drift/disturbances
                        double pidOmega = headingPID.calculate(
                                        swerve.getPose().getRotation().getRadians(),
                                        desiredFieldHeading.getRadians());

                        // Add the feedforward to actively drive the rotation
                        return pidOmega + feedforwardOmega;
                }
                return manualOmega;
        }

        public Rotation2d getDesiredRobotHeading() {
                return desiredFieldHeading;
        }

        public double getDistanceToHubMeters() {
                return distanceToHubMeters;
        }

        public boolean isHubTrackingEnabled() {
                return hubTrackingEnabled;
        }

        public SwerveSubsystem getSwerve() {
                return swerve;
        }

        // ------------------------------------------------
        // FUEL LOGIC
        // ------------------------------------------------
        public void intakeFuel() {
                if (fuelStored < FUEL_CAPACITY)
                        fuelStored++;
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
        }

        public void shoot() {
                if (!edu.wpi.first.wpilibj.RobotBase.isSimulation())
                        return;
                if (canShoot()) {
                        fuelStored--;
                        visualizer.fire(shooter.getTargetRPM(), hood.getTargetAngle());
                }
        }

        private void logToAdvantageScope() {
                SmartDashboard.putBoolean("Turret/HubTrackingEnabled", hubTrackingEnabled);
                SmartDashboard.putNumber("Targeting/DistanceMeters", distanceToHubMeters);
                SmartDashboard.putNumber("Targeting/DesiredHeadingDeg", desiredFieldHeading.getDegrees());
                SmartDashboard.putNumber("Targeting/RobotYawDeg", swerve.getPose().getRotation().getDegrees());
                SmartDashboard.putNumber("Targeting/HeadingErrorDeg",
                                desiredFieldHeading.minus(swerve.getPose().getRotation()).getDegrees());
                SmartDashboard.putNumber("Targeting/CalculatedRPM", calculatedRPM);
                SmartDashboard.putNumber("Targeting/CalculatedHoodDeg", calculatedHoodAngleDeg);
        }
}