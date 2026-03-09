package frc.robot.subsystems.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.util.HubTracker;

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
        private double distanceToTargetMeters = 0.0;
        private double calculatedRPM = 0.0;
        private double calculatedHoodAngleDeg = 0.0;

        // ------------------------------------------------
        // CONTROLLERS
        // ------------------------------------------------

        private final PIDController headingPID = new PIDController(1, 0.0, 0); // P=1.74 for smooth snapping
        private double feedforwardOmega = 0.0;
        // ------------------------------------------------
        // CONSTRUCTOR
        // ------------------------------------------------

        public TurretSubsystem(VisionSubsystem vision, SwerveSubsystem swerve, ShooterSubsystem shooter,
                        HoodSubsystem hood, FuelSim fuelSim) {
                this.vision = vision;
                this.swerve = swerve;
                this.shooter = shooter;
                this.hood = hood;
                this.fuelSim = fuelSim;

                headingPID.enableContinuousInput(-Math.PI, Math.PI);

                visualizer = new TurretVisualizer(fuelSim,
                                () -> new Pose3d(swerve.getPose().getTranslation().getX(),
                                                swerve.getPose().getTranslation().getY(), 0.0,
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
                // 1. Determine Alliance and Locations
                boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

                Translation2d hubPos = isBlue
                                ? Constants.FieldConstants.HUB_BLUE.toTranslation2d()
                                : Constants.FieldConstants.HUB_RED.toTranslation2d();

                Translation2d stockpilePos = isBlue
                                ? Constants.FieldConstants.STOCKPILE_BLUE
                                : Constants.FieldConstants.STOCKPILE_RED;

                // 2. AUTO-STOCKPILE LOGIC: Switch goal if Hub is inactive
                boolean hubIsActive = HubTracker.isActive();
                Translation2d currentGoal = hubIsActive ? hubPos : stockpilePos;

                // 3. GET ROBOT DATA
                Pose2d robotPose = swerve.getPose();
                ChassisSpeeds fieldSpeeds = swerve.getFieldVelocity();
                ChassisSpeeds robotSpeeds = swerve.getRobotVelocity();

                // 4. SOLVE USING THE "GOLD STANDARD" CALCULATOR
                // Pass both field and robot speeds for the "Whip Effect" math
                var solution = ShooterAimCalculator.solveMoving(
                                robotPose,
                                robotSpeeds,
                                fieldSpeeds,
                                currentGoal);

                // 5. STORE RESULTS
                distanceToTargetMeters = solution.distanceMeters();
                calculatedRPM = solution.rpm();
                desiredFieldHeading = solution.chassisHeading();

                // 6. KINEMATIC FEEDFORWARD (The active "Push")
                Translation2d robotToGoal = currentGoal.minus(robotPose.getTranslation());
                double rX = robotToGoal.getX();
                double rY = robotToGoal.getY();
                double vX = fieldSpeeds.vxMetersPerSecond;
                double vY = fieldSpeeds.vyMetersPerSecond;
                double distSq = (rX * rX) + (rY * rY);
                if (distSq > 0.01) {
                        feedforwardOmega = (rY * vX - rX * vY) / distSq;
                }

                if (hubTrackingEnabled) {
                        shooter.applyRPM(calculatedRPM);
                        hood.applyAngle(solution.hoodAngle());
                }

                SmartDashboard.putBoolean("Targeting/IsHubActive", hubIsActive);
                SmartDashboard.putBoolean("Targeting/IsStockpiling", !hubIsActive);
        }

        public double getDesiredRobotOmega() {
                if (hubTrackingEnabled) {
                        double pidOmega = headingPID.calculate(swerve.getPose().getRotation().getRadians(),
                                        desiredFieldHeading.getRadians());
                        return pidOmega + feedforwardOmega;
                }
                return manualOmega;
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
        }

        public double getCalculatedRPM() {
                return calculatedRPM;
        }

        public double getDistanceToHubMeters() {
                return distanceToTargetMeters;
        }

        public boolean isHubTrackingEnabled() {
                return hubTrackingEnabled;
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

        // Inside TurretSubsystem.java

        public Command shootCommand() {
                return run(() -> {
                        // Continue shooting logic
                        shoot();
                })
                                .beforeStarting(this::enableHubTracking) // Turn on tracking
                                .finallyDo(() -> {
                                        disableHubTracking(); // Turn off tracking on end/cancel
                                        shooter.stop(); // Optional: Stop wheels when done
                                });
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
                SmartDashboard.putNumber("Targeting/DistanceMeters", distanceToTargetMeters);
                SmartDashboard.putNumber("Targeting/DesiredHeadingDeg", desiredFieldHeading.getDegrees());
                SmartDashboard.putNumber("Targeting/RobotYawDeg", swerve.getPose().getRotation().getDegrees());
                SmartDashboard.putNumber("Targeting/HeadingErrorDeg",
                                desiredFieldHeading.minus(swerve.getPose().getRotation()).getDegrees());
                SmartDashboard.putNumber("Targeting/FeedforwardOmega", feedforwardOmega);
                SmartDashboard.putNumber("Targeting/CalculatedRPM", calculatedRPM);
        }
}