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
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterAimCalculator;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.Turret.TurretVisualizer;
import frc.robot.util.FuelSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.HubTracker;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import java.util.Optional;

public class TurretSubsystem extends SubsystemBase {

        private static final Transform2d robotToLauncher = VisionConstants.ROBOT_TO_LAUNCHER;
        private int fuelStored = 180;
        public static final int FUEL_CAPACITY = 200;

        private final VisionSubsystem vision;
        private final SwerveSubsystem swerve;
        private final ShooterSubsystem shooter;
        private final HoodSubsystem hood;
        private final FuelSim fuelSim;
        private final TurretVisualizer visualizer;

        private boolean hubTrackingEnabled = false;
        private double manualOmega = 0.0;
        private Rotation2d desiredFieldHeading = new Rotation2d();
        private double distanceToTargetMeters = 0.0;
        private double calculatedRPM = 0.0;
        private double feedforwardOmega = 0.0;

        // CONTROLLERS
        // Tuned for 4.5m/s: P=5.0 for strength
        private final PIDController headingPID = new PIDController(1, 0.0, 0);

        // Increased to 60.0 to allow instant reaction at high speeds
        private final SlewRateLimiter ffLimiter = new SlewRateLimiter(60.0);

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

        @Override
        public void periodic() {
                updateTargeting();
                SmartDashboard.putBoolean("Turret/IsTracking", hubTrackingEnabled);
                logToAdvantageScope();
                visualizer.update(shooter.getTargetRPM(), hood.getTargetAngle());
        }

        private void updateTargeting() {
                boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

                Translation2d hubPos = isBlue
                                ? Constants.FieldConstants.HUB_BLUE.toTranslation2d()
                                : Constants.FieldConstants.HUB_RED.toTranslation2d();

                Translation2d stockpilePos = isBlue
                                ? Constants.FieldConstants.STOCKPILE_BLUE
                                : Constants.FieldConstants.STOCKPILE_RED;

                // AUTO-STOCKPILE LOGIC
                boolean hubIsActive = HubTracker.isActive();
                Translation2d currentGoal = hubIsActive ? hubPos : hubPos;

                Pose2d robotPose = swerve.getPose();
                ChassisSpeeds fieldSpeeds = swerve.getFieldVelocity();
                ChassisSpeeds robotSpeeds = swerve.getRobotVelocity();

                // 1. KINEMATIC FEEDFORWARD
                Translation2d robotToGoal = currentGoal.minus(robotPose.getTranslation());
                double rX = robotToGoal.getX();
                double rY = robotToGoal.getY();
                double vX = fieldSpeeds.vxMetersPerSecond;
                double vY = fieldSpeeds.vyMetersPerSecond;
                double distSq = (rX * rX) + (rY * rY);

                double rawFF = 0;
                double velocityMagnitude = Math.sqrt(vX * vX + vY * vY);

                if (distSq > 0.01 && velocityMagnitude > 0.1) {
                        rawFF = (rY * vX - rX * vY) / distSq;
                }

                // Clamp FF to physical robot limit (approx 9 rad/s)
                rawFF = MathUtil.clamp(rawFF, -7.0, 7.0);
                feedforwardOmega = ffLimiter.calculate(rawFF);

                // 2. SOLVE USING THE "ELITE" CALCULATOR
                var params = ShooterAimCalculator.solveMoving(
                                robotPose,
                                robotSpeeds,
                                fieldSpeeds,
                                currentGoal,
                                hubIsActive);

                // 3. STORE RESULTS
                distanceToTargetMeters = params.distanceMeters();
                calculatedRPM = params.rpm();
                desiredFieldHeading = params.chassisHeading();

                if (hubTrackingEnabled) {
                        shooter.applyRPM(calculatedRPM);
                        hood.applyAngle(Radians.of(params.hoodAngle().in(Radians)));
                }

                SmartDashboard.putBoolean("Targeting/IsHubActive", hubIsActive);
                SmartDashboard.putBoolean("Targeting/IsStockpiling", !hubIsActive);
        }

        public double getDesiredRobotOmega() {
                if (hubTrackingEnabled) {
                        double pidOmega = headingPID.calculate(swerve.getPose().getRotation().getRadians(),
                                        desiredFieldHeading.getRadians());

                        // If error is > 30 degrees, reduce Feedforward to let PID recover
                        double error = Math.abs(desiredFieldHeading.minus(swerve.getPose().getRotation()).getDegrees());
                        double ffScalar = (error > 30) ? 0.5 : 1.0;

                        return pidOmega + (feedforwardOmega * ffScalar);
                }
                return manualOmega;
        }

        public Optional<Rotation2d> getRotationTargetOverride() {
                if (hubTrackingEnabled) {
                        return Optional.of(desiredFieldHeading);
                } else {
                        return Optional.empty();
                }
        }

        // Inside TurretSubsystem.java
        public double getHeadingErrorDegrees() {
                return desiredFieldHeading.minus(swerve.getPose().getRotation()).getDegrees();
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

        // FUEL & SIMULATION COMMANDS (DO NOT REMOVE)
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

        public void resetFuelStored() {
                fuelStored = 0;
        }

        public Command shootCommand() {
                return run(() -> {
                        shoot();
                })
                                // .beforeStarting(this::enableHubTracking)
                                .finallyDo(() -> {
                                        // disableHubTracking();
                                        shooter.stop();
                                });
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