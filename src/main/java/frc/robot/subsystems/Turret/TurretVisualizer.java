package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.*;

import frc.robot.Constants.FieldConstants;

import java.util.function.Supplier;

public class TurretVisualizer {

    // Shooter offset from robot center (FIXED SHOOTER)
    private static final Translation2d SHOOTER_OFFSET = new Translation2d(0.35, 0.0);

    private static final double SHOOTER_HEIGHT = 0.50; // meters
    private static final double GRAVITY = 9.81;

    private final Supplier<Pose3d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
    private final Supplier<Boolean> isBlueAlliance;

    // ---------------- NT PUBLISHERS ----------------
    private final StructArrayPublisher<Pose3d> trajectoryPub;
    private final StructPublisher<Pose3d> shooterPosePub;
    private final BooleanPublisher willHitPub;

    public TurretVisualizer(
            Supplier<Pose3d> robotPoseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier,
            Supplier<Boolean> isBlueAlliance) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;
        this.isBlueAlliance = isBlueAlliance;

        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        trajectoryPub = nt.getStructArrayTopic(
                "Shooter/Trajectory",
                Pose3d.struct).publish();

        shooterPosePub = nt.getStructTopic(
                "Shooter/3DPose",
                Pose3d.struct).publish();

        willHitPub = nt.getBooleanTopic(
                "Shooter/WillHit").publish();
    }

    // ------------------------------------------------
    // UPDATE
    // ------------------------------------------------
    public void update(
            LinearVelocity exitVelocity,
            Angle hoodAngle) {
        Pose3d robotPose = robotPoseSupplier.get();
        ChassisSpeeds speeds = fieldSpeedsSupplier.get();

        Rotation2d robotYaw = robotPose.getRotation().toRotation2d();

        Translation3d hub = isBlueAlliance.get()
                ? FieldConstants.HUB_BLUE
                : FieldConstants.HUB_RED;

        // Shooter position in field space
        Translation2d shooterXY = robotPose.getTranslation().toTranslation2d()
                .plus(SHOOTER_OFFSET.rotateBy(robotYaw));

        // Aim direction (robot → hub)
        Translation2d toHub = hub.toTranslation2d().minus(shooterXY);

        Rotation2d aimHeading = new Rotation2d(
                Math.atan2(toHub.getY(), toHub.getX()));

        // Initial velocities
        double v = exitVelocity.in(MetersPerSecond);
        double theta = hoodAngle.in(Radians);

        double vHorizontal = Math.cos(theta) * v;
        double vz = Math.sin(theta) * v;

        // Include robot motion (lead compensation)
        double vx = vHorizontal * aimHeading.getCos()
                + speeds.vxMetersPerSecond;

        double vy = vHorizontal * aimHeading.getSin()
                + speeds.vyMetersPerSecond;

        Pose3d[] trajectory = new Pose3d[50];
        boolean willHit = false;

        double funnelRadius = FieldConstants.FUNNEL_RADIUS.in(Meters);

        double funnelBottomZ = Inches.of(56.4).in(Meters);

        double funnelTopZ = Inches.of(72.0).in(Meters);

        for (int i = 0; i < trajectory.length; i++) {
            double t = i * 0.04;

            double x = shooterXY.getX() + vx * t;
            double y = shooterXY.getY() + vy * t;
            double z = SHOOTER_HEIGHT
                    + vz * t
                    - 0.5 * GRAVITY * t * t;

            trajectory[i] = new Pose3d(
                    x,
                    y,
                    Math.max(0.0, z),
                    new Rotation3d());

            if (!willHit
                    && z >= funnelBottomZ
                    && z <= funnelTopZ
                    && new Translation2d(x, y)
                            .getDistance(hub.toTranslation2d()) <= funnelRadius) {
                willHit = true;
            }
        }

        // ---------------- PUBLISH ----------------
        trajectoryPub.set(trajectory);
        willHitPub.set(willHit);

        shooterPosePub.set(
                new Pose3d(
                        shooterXY.getX(),
                        shooterXY.getY(),
                        SHOOTER_HEIGHT,
                        new Rotation3d(
                                0, 0, aimHeading.getRadians())));
    }
}
