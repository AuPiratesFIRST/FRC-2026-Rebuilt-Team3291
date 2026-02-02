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
 * - Shape RPM using logarithmic acceleration
 *
 * DOES NOT:
 * - Control motors
 * - Schedule commands
 * - Use vision subsystems directly
 */
public final class ShooterAimCalculator {

    // ============================================================
    // FIELD + MECHANISM CONSTANTS
    // ============================================================

    /** Shooter exit height (meters) */
    public static final double SHOOTER_HEIGHT = 0.50;

    /** Target scoring height (meters) */
    public static final double TARGET_HEIGHT = 2.64;

    /** Shooter wheel radius (meters) */
    public static final double WHEEL_RADIUS = Inches.of(4).in(Meters);

    /** Gravity (m/s^2) */
    private static final double GRAVITY = 9.81;

    /** Absolute RPM safety clamp */
    public static final double MAX_RPM = 5500.0;

    /** Valid shooting distance bounds */
    private static final double MIN_DISTANCE = 0.1;
    private static final double MAX_DISTANCE = 3.5;

    // ============================================================
    // LOGARITHMIC RPM SHAPING
    // ============================================================

    /** Log curve aggressiveness (higher = steeper rise) */
    private static final double LOG_K = 6.0;

    // ============================================================
    // TUNING MAPS (PRIMARY CONTROL SOURCE)
    // ============================================================

    private static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
        // Distance (m) -> Flywheel RPM (RAW, BEFORE log shaping)
        rpmMap.put(0.10, 1000.0);
        rpmMap.put(0.20, 2000.0);
        rpmMap.put(0.25, 3000.0);
        rpmMap.put(0.27, 4000.0);
        rpmMap.put(0.30, 5000.0);

        // Distance (m) -> Hood angle (deg)
        hoodAngleMap.put(0.10, 25.0);
        hoodAngleMap.put(0.20, 30.0);
        hoodAngleMap.put(0.25, 35.0);
        hoodAngleMap.put(0.27, 40.0);
        hoodAngleMap.put(0.30, 45.0);
    }

    // ============================================================
    // PUBLIC SOLVER
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

        // RAW RPM from table (you control this)
        double rawRPM = Math.min(rpmMap.get(clamped), MAX_RPM);

        // 🔥 Logarithmic shaping
        double shapedRPM = applyLogCurve(rawRPM);

        double physicsRPM = calculatePhysicsRPM(clamped, hoodAngle);

        return new ShooterSolution(
                hoodAngle,
                shapedRPM,
                physicsRPM,
                clamped,
                true);
    }

    /**
     * Fallback solution (YOU set the RPM here)
     */
    public static ShooterSolution fallback() {

        double rawRPM = 500.0;
        double shapedRPM = applyLogCurve(rawRPM);

        return new ShooterSolution(
                Degrees.of(35),
                shapedRPM,
                0.0,
                0.0,
                true);
    }

    // ============================================================
    // LOGARITHMIC CURVE (CORE LOGIC)
    // ============================================================

    private static double applyLogCurve(double rpm) {

        double normalized = MathUtil.clamp(rpm / MAX_RPM, 0.0, 1.0);

        double curved = Math.log(1.0 + LOG_K * normalized) /
                Math.log(1.0 + LOG_K);

        return MAX_RPM * curved;
    }

    // ============================================================
    // PHYSICS MODEL (VALIDATION ONLY)
    // ============================================================

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

    private ShooterAimCalculator() {
    }
}
