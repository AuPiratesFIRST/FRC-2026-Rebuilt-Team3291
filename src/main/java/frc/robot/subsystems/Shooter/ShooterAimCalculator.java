package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
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

    // Latency: Time for ball to leave the robot (Phase delay)
    private static final double SYSTEM_LATENCY_SEC = 0.35;

    // ============================================================
    // TUNING MAPS
    // ============================================================

    private static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    // NEW: Time of Flight Map (Distance in meters -> Time in seconds)
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
        /**
         * Distance (m) -> Flywheel RPM
         * Based on Anchors: 1.5m @ 1150, 2.0m @ 1200
         */
        rpmMap.put(1.20, 1075.0); // Slightly less than 1.5m, but high angle needs speed
        rpmMap.put(1.50, 1150.0); // TESTED ANCHOR
        rpmMap.put(2.00, 1200.0); // TESTED ANCHOR
        rpmMap.put(2.50, 1348.0); // Curve starts to steepen here
        rpmMap.put(3.00, 1359.0); // REDUCED from 1550 to prevent overshot
        rpmMap.put(4.00, 1460.0); // REDUCED from 2100 to prevent overshot
        rpmMap.put(4.25, 1540.0);
        rpmMap.put(4.50, 1590.0);

        /**
         * Distance (m) -> Hood angle (deg)
         * Based on Anchors: 1.5m @ 65°, 2.0m @ 59°
         */
        hoodAngleMap.put(1.20, 64.09);
        hoodAngleMap.put(1.50, 64.09);
        hoodAngleMap.put(2.00, 64.09);
        hoodAngleMap.put(2.50, 64.09);
        hoodAngleMap.put(3.00, 64.09);
        hoodAngleMap.put(4.00, 64.09);

        /**
         * Distance (m) -> Time of Flight (seconds)
         * TUNE THESE! These are baseline estimates. If your ball misses "behind" your
         * movement direction, increase these times. If it misses "ahead", decrease
         * them.
         */
        timeOfFlightMap.put(1.20, 0.40);
        timeOfFlightMap.put(1.50, 0.48);
        timeOfFlightMap.put(2.00, 0.58);
        timeOfFlightMap.put(2.50, 0.65);
        timeOfFlightMap.put(3.00, 0.72);
        timeOfFlightMap.put(4.00, 0.85);
        timeOfFlightMap.put(4.50, 0.95);
    }

    // ============================================================
    // PUBLIC SOLVER (MOVING)
    // ============================================================
    public static MovingShotSolution solveMoving(
            Pose2d robotPose,
            ChassisSpeeds fieldSpeeds,
            Translation2d goalLocation) {

        // 1. PROJECT ROBOT POSE (Phase Delay)
        // Project where the robot will be in 35ms (SYSTEM_LATENCY_SEC)
        // Using exponential move (Twist2d) is more accurate for swerve than linear math
        Pose2d predictedPose = robotPose.exp(new Twist2d(
                fieldSpeeds.vxMetersPerSecond * SYSTEM_LATENCY_SEC,
                fieldSpeeds.vyMetersPerSecond * SYSTEM_LATENCY_SEC,
                fieldSpeeds.omegaRadiansPerSecond * SYSTEM_LATENCY_SEC));

        // 2. CALCULATE SHOOTER VELOCITY (The "Whip Effect")
        // The shooter is at an offset. If the robot is rotating, the shooter's field
        // velocity
        // is different from the chassis center's velocity.
        double rx = 0.35; // SHOOTER_OFFSET X
        double ry = 0.0; // SHOOTER_OFFSET Y
        double omega = fieldSpeeds.omegaRadiansPerSecond;

        double shooterFieldVx = fieldSpeeds.vxMetersPerSecond - (omega * ry);
        double shooterFieldVy = fieldSpeeds.vyMetersPerSecond + (omega * rx);
        Translation2d shooterVelocity = new Translation2d(shooterFieldVx, shooterFieldVy);

        // 3. ITERATIVE CONVERGENCE (The 20-Loop)
        Translation2d shooterPos = predictedPose.getTranslation().plus(
                new Translation2d(rx, ry).rotateBy(predictedPose.getRotation()));

        double distanceToHub = shooterPos.getDistance(goalLocation);
        Translation2d virtualGoal = goalLocation;

        for (int i = 0; i < 20; i++) {
            double clampedDist = MathUtil.clamp(distanceToHub, MIN_DISTANCE, MAX_DISTANCE);
            double tof = timeOfFlightMap.get(clampedDist);

            // Move the goal in the opposite direction of our shooter's movement
            virtualGoal = goalLocation.minus(shooterVelocity.times(tof));
            distanceToHub = shooterPos.getDistance(virtualGoal);
        }

        // 4. OUTPUTS
        Rotation2d ballFlightAngle = virtualGoal.minus(shooterPos).getAngle();

        // THE 180-FLIP: Since shooter is on back, front points AWAY from ball flight
        Rotation2d chassisHeading = ballFlightAngle.plus(Rotation2d.fromDegrees(180));

        double finalDist = shooterPos.getDistance(virtualGoal);
        double rpm = rpmMap.get(finalDist);
        Angle hoodAngle = Degrees.of(hoodAngleMap.get(finalDist));

        return new MovingShotSolution(chassisHeading, rpm, hoodAngle, finalDist);
    }

    // Update the Record definition at the bottom or top of the class
    public record MovingShotSolution(Rotation2d chassisHeading, double rpm, Angle hoodAngle, double distanceMeters) {
    }

    // Update the method signature to accept 4 arguments
    public static MovingShotSolution solveMoving(
            Pose2d robotPose,
            ChassisSpeeds robotRelativeSpeeds, // NEW
            ChassisSpeeds fieldSpeeds, // NEW
            Translation2d goalLocation) {

        // 1. PROJECT ROBOT POSE (Phase Delay)
        // Project where the robot will be in 35ms (SYSTEM_LATENCY_SEC)
        Pose2d predictedPose = robotPose.exp(new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * SYSTEM_LATENCY_SEC,
                robotRelativeSpeeds.vyMetersPerSecond * SYSTEM_LATENCY_SEC,
                robotRelativeSpeeds.omegaRadiansPerSecond * SYSTEM_LATENCY_SEC));

        // 2. CALCULATE SHOOTER VELOCITY (The "Whip Effect")
        double rx = 0.35; // The X offset of the shooter
        double ry = 0.0;
        double omega = fieldSpeeds.omegaRadiansPerSecond;

        double shooterFieldVx = fieldSpeeds.vxMetersPerSecond - (omega * ry);
        double shooterFieldVy = fieldSpeeds.vyMetersPerSecond + (omega * rx);
        Translation2d shooterVelocity = new Translation2d(shooterFieldVx, shooterFieldVy);

        // 3. ITERATIVE CONVERGENCE (The 20-Loop)
        Translation2d shooterPos = predictedPose.getTranslation().plus(
                new Translation2d(rx, ry).rotateBy(predictedPose.getRotation()));

        double distanceToHub = shooterPos.getDistance(goalLocation);
        Translation2d virtualGoal = goalLocation;

        for (int i = 0; i < 20; i++) {
            double clampedDist = MathUtil.clamp(distanceToHub, MIN_DISTANCE, MAX_DISTANCE);
            double tof = timeOfFlightMap.get(clampedDist);

            virtualGoal = goalLocation.minus(shooterVelocity.times(tof));
            distanceToHub = shooterPos.getDistance(virtualGoal);
        }

        // 4. OUTPUTS
        Rotation2d ballFlightAngle = virtualGoal.minus(shooterPos).getAngle();
        Rotation2d chassisHeading = ballFlightAngle.plus(Rotation2d.fromDegrees(180));

        double finalDist = shooterPos.getDistance(virtualGoal);
        double rpm = rpmMap.get(finalDist);
        Angle hoodAngle = Degrees.of(hoodAngleMap.get(finalDist));

        return new MovingShotSolution(chassisHeading, rpm, hoodAngle, finalDist);
    }

    // ============================================================
    // PUBLIC SOLVER (STATIONARY)
    // ============================================================
    public static ShooterSolution solve(double distanceMeters) {

        double clamped = MathUtil.clamp(distanceMeters, MIN_DISTANCE, MAX_DISTANCE);
        Angle hoodAngle = Degrees.of(hoodAngleMap.get(clamped));

        double targetRPM = Math.min(rpmMap.get(clamped), MAX_RPM);

        return new ShooterSolution(hoodAngle, targetRPM, 0.0, clamped, true);
    }

    public static ShooterSolution fallback() {
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