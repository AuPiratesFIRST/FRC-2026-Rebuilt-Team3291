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
    public static final double MAX_RPM = 6000.0;

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
    // These maps are the MAIN TUNING INTERFACE for shooters.
    // Add distance→RPM/angle pairs from testing, and the calculator
    // will automatically interpolate between them.
    //
    // InterpolatingDoubleTreeMap does LINEAR INTERPOLATION between points:
    // Example: If you have (0.10m → 1000 RPM) and (0.20m → 2000 RPM)
    // and ask for 0.15m, you get 1500 RPM
    //
    // HOW TO TUNE:
    // 1. Stand robot at a known distance from hub
    // 2. Manually adjust RPM and hood angle until shots score
    // 3. Record the distance, RPM, and angle
    // 4. Add that data point below
    // 5. Repeat at multiple distances (close, medium, far)
    // 6. The calculator handles everything in between!

    private static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
        // Distance (m) → Flywheel RPM (RAW, BEFORE log shaping)
        // These are STARTING VALUES - tune based on real testing!
        rpmMap.put(0.10, 1000.0); // Very close shot
        rpmMap.put(0.20, 2000.0); // Short distance
        rpmMap.put(0.25, 3000.0); // Medium distance
        rpmMap.put(1.27, 4000.0); // Longer shot
        rpmMap.put(2.30, 5000.0); // Maximum range

        // Distance (m) → Hood angle (degrees)
        // Lower angles = flatter trajectory (close shots)
        // Higher angles = arced trajectory (far shots)
        hoodAngleMap.put(0.10, 25.0); // Low angle for close range
        hoodAngleMap.put(0.20, 30.0);
        hoodAngleMap.put(0.25, 35.0);
        hoodAngleMap.put(0.27, 40.0);
        hoodAngleMap.put(0.30, 45.0); // High angle for long range
    }

    // ============================================================
    // PUBLIC SOLVER
    // ============================================================

    /**
     * Computes a complete shooter solution from distance alone.
     * 
     * This is the main entry point called by commands. Give it a distance,
     * get back RPM and hood angle settings.
     * 
     * Process:
     * 1. Check if distance is in valid range
     * 2. Look up RPM and angle from interpolation maps
     * 3. Apply logarithmic shaping to RPM (smooth acceleration curve)
     * 4. Calculate physics-based RPM for comparison/validation
     * 5. Return complete solution
     * 
     * @param distanceMeters Horizontal distance to target (from vision)
     * @return ShooterSolution containing RPM, hood angle, and validity flag
     */
    public static ShooterSolution solve(double distanceMeters) {

        // Validate distance is within our tested range
        if (distanceMeters < MIN_DISTANCE || distanceMeters > MAX_DISTANCE) {
            return ShooterSolution.invalid(distanceMeters);
        }

        // Clamp just in case (defensive programming)
        double clamped = MathUtil.clamp(distanceMeters, MIN_DISTANCE, MAX_DISTANCE);

        // Look up hood angle from interpolation table
        // InterpolatingDoubleTreeMap automatically interpolates between known points
        Angle hoodAngle = Degrees.of(hoodAngleMap.get(clamped));

        // Look up RAW RPM from table (before curve shaping)
        // This is what YOU tuned during testing
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

        double rawRPM = 0.0;
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
