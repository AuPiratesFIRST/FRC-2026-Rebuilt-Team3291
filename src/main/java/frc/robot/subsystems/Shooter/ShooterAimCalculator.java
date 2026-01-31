package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * ShooterAimCalculator
 * ------------------------------------------------------------
 * Centralized math + tuning logic for vision-based shooting.
 *
 * RESPONSIBILITIES:
 * - Convert distance (meters) -> hood angle
 * - Convert distance (meters) -> flywheel RPM
 * - Optionally validate RPM using projectile physics
 *
 * DOES NOT:
 * - Control motors
 * - Schedule commands
 * - Use vision subsystems directly
 *  
 * SAFE TO USE FROM:
 * - Commands
 * - Autos
 * - Subsystems
 */
public final class ShooterAimCalculator {

    // ============================================================
    // FIELD + MECHANISM CONSTANTS
    // ============================================================

    /** Shooter exit height (meters) */
    public static final double SHOOTER_HEIGHT = 0.50;

    /** Target scoring height (meters) ~12" above upper AprilTag */
    public static final double TARGET_HEIGHT = 2.64;

    /** Shooter wheel radius (meters) */
    public static final double WHEEL_RADIUS = Inches.of(4).in(Meters);

    /** Gravity (m/s^2) */
    private static final double GRAVITY = 9.81;

    /** Absolute RPM safety clamp */
    public static final double MAX_RPM = 6000.0;

    /** Valid shooting distance bounds */
    private static final double MIN_DISTANCE = 1.5;
    private static final double MAX_DISTANCE = 3.5;

    // ============================================================
    // TUNING MAPS (PRIMARY CONTROL SOURCE)
    // ============================================================

    private static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
        // Distance (m) -> Flywheel RPM
        rpmMap.put(1.5, 2500.0);
        rpmMap.put(2.0, 3000.0);
        rpmMap.put(2.5, 3500.0);
        rpmMap.put(3.0, 4200.0);
        rpmMap.put(3.5, 4800.0);

        // Distance (m) -> Hood angle (deg)
        hoodAngleMap.put(1.5, 25.0);
        hoodAngleMap.put(2.0, 30.0);
        hoodAngleMap.put(2.5, 35.0);
        hoodAngleMap.put(3.0, 40.0);
        hoodAngleMap.put(3.5, 45.0);
    }

    // ============================================================
    // PUBLIC SOLVER (USED BY COMMANDS)
    // ============================================================

    /**
     * Computes a complete shooter solution from distance alone.
     */
    public static ShooterSolution solve(double distanceMeters) {

        if (distanceMeters < MIN_DISTANCE || distanceMeters > MAX_DISTANCE) {
            return ShooterSolution.invalid(distanceMeters);
        }

        double clamped = MathUtil.clamp(distanceMeters, MIN_DISTANCE, MAX_DISTANCE);

        Angle hoodAngle = Degrees.of(hoodAngleMap.get(clamped));

        double rpm = Math.min(rpmMap.get(clamped), MAX_RPM);

        // Optional physics validation (non-authoritative)
        double physicsRPM = calculatePhysicsRPM(clamped, hoodAngle);

        return new ShooterSolution(
                hoodAngle,
                rpm,
                physicsRPM,
                clamped,
                true);
    }

    // ============================================================
    // PHYSICS MODEL (VALIDATION / FUTURE AUTO-TUNING)
    // ============================================================

    /**
     * Calculates RPM using projectile motion.
     * NOT used directly for control — tuning table wins.
     */
    private static double calculatePhysicsRPM(
            double distanceMeters,
            Angle hoodAngle) {
        double theta = hoodAngle.in(Radians);
        double d = distanceMeters;
        double h = TARGET_HEIGHT - SHOOTER_HEIGHT;

        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double denom = 2.0 * cos * cos * (h - d * tan);

        if (denom >= 0.0) {
            return 0.0;
        }

        double v = Math.sqrt((GRAVITY * d * d) / (-denom));

        double rpm = (v / WHEEL_RADIUS) * 60.0 / (2.0 * Math.PI);

        return Math.min(rpm, MAX_RPM);
    }

    // ============================================================
    // PHOTONVISION UTILITY
    // ============================================================

    /**
     * Extracts horizontal distance (meters) from a Photon target.
     */
    public static double extractPlanarDistance(
            PhotonTrackedTarget target) {
        Transform3d camToTarget = target.getBestCameraToTarget();

        double x = camToTarget.getTranslation().getX();
        double y = camToTarget.getTranslation().getY();

        return Math.hypot(x, y);
    }

    // ============================================================
    // RESULT CONTAINER
    // ============================================================

    public record ShooterSolution(
            Angle hoodAngle,
            double rpm,
            double physicsRPM,
            double distanceMeters,
            boolean valid) {
        public static ShooterSolution invalid(double distance) {
            return new ShooterSolution(
                    Degrees.zero(),
                    0.0,
                    0.0,
                    distance,
                    false);
        }
    }

    // Prevent construction
    private ShooterAimCalculator() {
    }
}
