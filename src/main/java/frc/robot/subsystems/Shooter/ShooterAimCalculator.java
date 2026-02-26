package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;

public final class ShooterAimCalculator {

    // ============================================================
    // FIELD + MECHANISM CONSTANTS
    // ============================================================

    public static final double SHOOTER_HEIGHT = 0.50;
    public static final double TARGET_HEIGHT = 1.83; // MATCHED TO GAME MANUAL PAGE 23
    public static final double WHEEL_RADIUS = Inches.of(4).in(Meters);
    private static final double GRAVITY = 9.81;

    public static final double MAX_RPM = 5000.0;

    // UPDATED: Min distance is 1.2m because of robot size + hub base
    private static final double MIN_DISTANCE = 1.2;
    private static final double MAX_DISTANCE = 4.5;

    // Latency: Time for ball to leave the robot
    private static final double SYSTEM_LATENCY_SEC = 0.25;

    // ============================================================
    // TUNING MAPS
    // ============================================================

    private static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
        /**
         * Distance (m) -> Flywheel RPM
         * Based on Anchors: 1.5m @ 1150, 2.0m @ 1200
         */
        rpmMap.put(1.20, 1075.0); // Slightly less than 1.5m, but high angle needs speed
        rpmMap.put(1.50, 1150.0); // TESTED ANCHOR
        rpmMap.put(2.00, 1200.0); // TESTED ANCHOR
        rpmMap.put(2.50, 1348.0); // Curve starts to steepen here
        rpmMap.put(3.00, 1280.0); // REDUCED from 1550 to prevent overshot
        rpmMap.put(4.00, 1850.0); // REDUCED from 2100 to prevent overshot

        /**
         * Distance (m) -> Hood angle (deg)
         * Based on Anchors: 1.5m @ 65°, 2.0m @ 59°
         */
        hoodAngleMap.put(1.20, 72.0); // Steeper for the close-in "drop"
        hoodAngleMap.put(1.50, 65.0); // TESTED ANCHOR
        hoodAngleMap.put(2.00, 59.0); // TESTED ANCHOR
        hoodAngleMap.put(2.50, 53.0); // Scaling down the angle to reach
        hoodAngleMap.put(3.00, 59.0); // RAISED from 48.0 (Steeper = loops into goal)
        hoodAngleMap.put(4.00, 48.0); // RAISED from 40.0 (The physics "sweet spot")
    }

    // ============================================================
    // PUBLIC SOLVER (MOVING)
    // ============================================================
    public static MovingShotSolution solveMoving(
            Pose2d shooterPose,
            ChassisSpeeds fieldSpeeds,
            Translation2d goalLocation) {

        Translation2d robotVelocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

        // 1. Predict robot position when the ball actually leaves the shooter
        Translation2d predictedShooterPos = shooterPose.getTranslation().plus(robotVelocity.times(SYSTEM_LATENCY_SEC));

        // 2. Calculate distance and base requirements from the LUT
        Translation2d targetVec = goalLocation.minus(predictedShooterPos);
        double dist = MathUtil.clamp(targetVec.getNorm(), MIN_DISTANCE, MAX_DISTANCE);

        double idealStationaryRPM = rpmMap.get(dist);
        Angle hoodAngle = Degrees.of(hoodAngleMap.get(dist));

        // 3. Vector Compensation: Subtract robot velocity from the required exit
        // velocity vector
        double tangentialVelocityMS = (idealStationaryRPM / 60.0) * (2 * Math.PI * WHEEL_RADIUS);
        Translation2d exitVelocityVec = targetVec.div(targetVec.getNorm()).times(tangentialVelocityMS);
        Translation2d compensatedVec = exitVelocityVec.minus(robotVelocity);

        // 4. Convert back to RPM and Heading
        Rotation2d compensatedHeading = compensatedVec.getAngle();
        double compensatedRPM = (compensatedVec.getNorm() * 60.0) / (2 * Math.PI * WHEEL_RADIUS);

        return new MovingShotSolution(
                compensatedHeading,
                MathUtil.clamp(compensatedRPM, 0, MAX_RPM),
                hoodAngle,
                dist);
    }

    public record MovingShotSolution(Rotation2d heading, double rpm, Angle hoodAngle, double distanceMeters) {
    }

    // ============================================================
    // PUBLIC SOLVER (STATIONARY)
    // ============================================================
    public static ShooterSolution solve(double distanceMeters) {

        double clamped = MathUtil.clamp(distanceMeters, MIN_DISTANCE, MAX_DISTANCE);
        Angle hoodAngle = Degrees.of(hoodAngleMap.get(clamped));

        // FIX: Removed applyLogCurve. We use the raw value from the map.
        double targetRPM = Math.min(rpmMap.get(clamped), MAX_RPM);

        return new ShooterSolution(hoodAngle, targetRPM, 0.0, clamped, true);
    }

    public static ShooterSolution fallback() {
        // FIX: Replaced shapedRPM with raw 1150 value
        return new ShooterSolution(Degrees.of(65), 1150.0, 0.0, 1.5, true);
    }

    // result container
    public record ShooterSolution(Angle hoodAngle, double rpm, double physicsRPM, double distanceMeters,
            boolean valid) {
        public static ShooterSolution invalid(double distance) {
            return new ShooterSolution(Degrees.zero(), 0.0, 0.0, distance, false);
        }
    }

    private ShooterAimCalculator() {
    }
}