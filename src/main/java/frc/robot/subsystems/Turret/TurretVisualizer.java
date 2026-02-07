package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.FuelSim;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretVisualizer {

        // ------------------------------------------------
        // CONSTANTS
        // ------------------------------------------------

        private static final Translation2d SHOOTER_OFFSET = new Translation2d(0.35, 0.0);

        private static final double SHOOTER_HEIGHT = 0.50; // meters
        private static final double GRAVITY = 9.81;
        private static final double DT = 0.04;
        private static final int TRAJ_LEN = 50;

        // 4" wheel
        private static final double WHEEL_RADIUS = Inches.of(2.0).in(Meters);

        // ------------------------------------------------
        // DEPENDENCIES
        // ------------------------------------------------

        private final FuelSim fuelSim;
        private final Supplier<Pose3d> robotPoseSupplier;
        private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
        private final Supplier<Boolean> isBlueAlliance;

        // ------------------------------------------------
        // STATE
        // ------------------------------------------------

        private final Pose3d[] trajectory = new Pose3d[TRAJ_LEN];

        // ------------------------------------------------
        // NT PUBLISHERS
        // ------------------------------------------------

        private final StructArrayPublisher<Pose3d> trajectoryPub;
        private final StructPublisher<Pose3d> shooterPosePub;
        private final BooleanPublisher willHitPub;

        // ------------------------------------------------
        // CONSTRUCTOR
        // ------------------------------------------------

        public TurretVisualizer(
                        FuelSim fuelSim,
                        Supplier<Pose3d> robotPoseSupplier,
                        Supplier<ChassisSpeeds> fieldSpeedsSupplier,
                        Supplier<Boolean> isBlueAlliance) {

                this.fuelSim = fuelSim;
                this.robotPoseSupplier = robotPoseSupplier;
                this.fieldSpeedsSupplier = fieldSpeedsSupplier;
                this.isBlueAlliance = isBlueAlliance;

                NetworkTableInstance nt = NetworkTableInstance.getDefault();

                trajectoryPub = nt.getStructArrayTopic(
                                "Shooter/Trajectory", Pose3d.struct).publish();

                shooterPosePub = nt.getStructTopic(
                                "Shooter/ShooterPose", Pose3d.struct).publish();

                willHitPub = nt.getBooleanTopic(
                                "Shooter/WillHit").publish();
        }

        // ------------------------------------------------
        // VISUAL TRAJECTORY ONLY
        // ------------------------------------------------

        public void update(double targetRPM, Angle hoodAngle) {

                Pose3d robotPose = robotPoseSupplier.get();
                ChassisSpeeds speeds = fieldSpeedsSupplier.get();

                Rotation2d robotYaw = robotPose.getRotation().toRotation2d();

                Translation3d hub = isBlueAlliance.get()
                                ? FieldConstants.HUB_BLUE
                                : FieldConstants.HUB_RED;

                Translation2d shooterXY = robotPose.getTranslation().toTranslation2d()
                                .plus(SHOOTER_OFFSET.rotateBy(robotYaw));

                Translation2d toHub = hub.toTranslation2d().minus(shooterXY);

                Rotation2d aimHeading = new Rotation2d(Math.atan2(
                                toHub.getY(), toHub.getX()));

                double exitVelocity = (targetRPM / 60.0)
                                * (2.0 * Math.PI * WHEEL_RADIUS);

                double theta = Math.max(
                                hoodAngle.in(Radians),
                                Degrees.of(5).in(Radians));

                double vHorizontal = Math.cos(theta) * exitVelocity;
                double vz = Math.sin(theta) * exitVelocity;

                double vx = vHorizontal * aimHeading.getCos()
                                + speeds.vxMetersPerSecond;

                double vy = vHorizontal * aimHeading.getSin()
                                + speeds.vyMetersPerSecond;

                double funnelRadius = FieldConstants.FUNNEL_RADIUS.in(Meters);

                double funnelBottomZ = Inches.of(56.4).in(Meters);

                double funnelTopZ = Inches.of(72.0).in(Meters);

                boolean willHit = false;

                for (int i = 0; i < TRAJ_LEN; i++) {
                        double t = i * DT;

                        double x = shooterXY.getX() + vx * t;
                        double y = shooterXY.getY() + vy * t;
                        double z = SHOOTER_HEIGHT
                                        + vz * t
                                        - 0.5 * GRAVITY * t * t;

                        trajectory[i] = new Pose3d(
                                        x, y, Math.max(0.0, z),
                                        new Rotation3d(0.0, -theta,
                                                        aimHeading.getRadians()));

                        if (!willHit
                                        && z >= funnelBottomZ
                                        && z <= funnelTopZ
                                        && new Translation2d(x, y)
                                                        .getDistance(hub.toTranslation2d()) <= funnelRadius) {
                                willHit = true;
                        }
                }

                trajectoryPub.set(trajectory);
                willHitPub.set(willHit);

                shooterPosePub.set(new Pose3d(
                                shooterXY.getX(),
                                shooterXY.getY(),
                                SHOOTER_HEIGHT,
                                new Rotation3d(0.0, 0.0,
                                                aimHeading.getRadians())));

                Logger.recordOutput("Shooter/Trajectory", trajectory);
                Logger.recordOutput("Shooter/WillHit", willHit);
        }

        // ------------------------------------------------
        // AUTHORITATIVE FUEL SIM FIRE
        // ------------------------------------------------

        public void fire(double targetRPM, Angle hoodAngle) {

                if (targetRPM <= 0)
                        return;

                Pose3d robotPose = robotPoseSupplier.get();
                ChassisSpeeds speeds = fieldSpeedsSupplier.get(); // Get current robot field-centric speeds
                Rotation2d robotYaw = robotPose.getRotation().toRotation2d();

                Translation3d hub = isBlueAlliance.get()
                                ? FieldConstants.HUB_BLUE
                                : FieldConstants.HUB_RED;

                // Calculate the shooter's actual field-centric horizontal position
                Translation2d shooterXY = robotPose.getTranslation().toTranslation2d()
                                .plus(SHOOTER_OFFSET.rotateBy(robotYaw));

                // Calculate the aiming heading towards the hub from the shooter's position
                Translation2d toHub = hub.toTranslation2d().minus(shooterXY);
                Rotation2d aimHeading = new Rotation2d(Math.atan2(
                                toHub.getY(), toHub.getX()));

                // Calculate exit velocity magnitude
                double exitVelocity = (targetRPM / 60.0)
                                * (2.0 * Math.PI * WHEEL_RADIUS);

                // Calculate hood angle, ensuring a minimum for physics
                double theta = Math.max(
                                hoodAngle.in(Radians),
                                Degrees.of(5).in(Radians));

                // Decompose exit velocity into horizontal and vertical components relative to
                // shooter's aim
                double vHorizontal = Math.cos(theta) * exitVelocity;
                double vz = Math.sin(theta) * exitVelocity;

                // Combine shooter's horizontal velocity with robot's field-centric speeds
                // This accounts for the robot's motion affecting the projectile's initial
                // velocity
                double vx = vHorizontal * aimHeading.getCos()
                                + speeds.vxMetersPerSecond; // Add robot's forward/backward speed
                double vy = vHorizontal * aimHeading.getSin()
                                + speeds.vyMetersPerSecond; // Add robot's strafe speed

                // Initial position of the fuel
                Translation3d initialPos = new Translation3d(
                                shooterXY.getX(),
                                shooterXY.getY(),
                                SHOOTER_HEIGHT);

                // Initial velocity vector of the fuel
                Translation3d initialVel = new Translation3d(vx, vy, vz);

                fuelSim.spawnFuel(initialPos, initialVel); // Use spawnFuel with calculated initial position and
                                                           // velocity
        }
}