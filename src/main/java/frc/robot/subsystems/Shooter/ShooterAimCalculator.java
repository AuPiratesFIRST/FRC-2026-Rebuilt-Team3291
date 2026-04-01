package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

public final class ShooterAimCalculator {

    // Shooter is 0.35m front the center, facing the back (180 deg)
    private static final Transform2d robotToLauncher = VisionConstants.ROBOT_TO_LAUNCHER;

    private static final double PHASE_DELAY = 0.03;
    private static final double MIN_DIST = 0.9;
    private static final double MAX_DIST = 5.6;
    public static final double MAX_RPM = 5000.0;

    private static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap stockpileRpmMap = new InterpolatingDoubleTreeMap();

    static {
        rpmMap.put(1.00, 2800.0);
        rpmMap.put(1.20, 2835.0);
        rpmMap.put(1.50, 2960.0);
        rpmMap.put(2.00, 3210.0);
        rpmMap.put(2.50, 3350.0);
        rpmMap.put(3.00, 3520.0);
        rpmMap.put(4.00, 4260.0);
        // rpmMap.put(4.25, 4540.0);
        // rpmMap.put(4.50, 4590.0);

        hoodAngleMap.put(1.20, 64.09);
        hoodAngleMap.put(1.50, 64.09);
        hoodAngleMap.put(2.00, 64.09);
        hoodAngleMap.put(2.50, 64.09);
        hoodAngleMap.put(3.00, 64.09);
        hoodAngleMap.put(4.00, 64.09);

        timeOfFlightMap.put(1.20, 0.40);
        timeOfFlightMap.put(1.50, 0.48);
        timeOfFlightMap.put(2.00, 0.58);
        timeOfFlightMap.put(2.50, 0.65);
        timeOfFlightMap.put(3.00, 0.72);
        timeOfFlightMap.put(4.00, 0.85);
        timeOfFlightMap.put(4.50, 0.95);

        stockpileRpmMap.put(2.0, 2800.0);
        stockpileRpmMap.put(3.0, 3900.0);
        stockpileRpmMap.put(4.0, 4900.0);

    }

    public record MovingShotSolution(Rotation2d chassisHeading, double rpm, Angle hoodAngle, double distanceMeters) {
    }

    public static MovingShotSolution solveMoving(
            Pose2d robotPose,
            ChassisSpeeds robotRelativeSpeeds,
            ChassisSpeeds fieldSpeeds,
            Translation2d goalLocation, boolean isHubTarget) {

        // A. PREDICT FUTURE POSE (30ms phase delay)
        Pose2d predictedPose = robotPose.exp(new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * PHASE_DELAY,
                robotRelativeSpeeds.vyMetersPerSecond * PHASE_DELAY,
                robotRelativeSpeeds.omegaRadiansPerSecond * PHASE_DELAY));

        // B. WHIP EFFECT: Calculate field velocity of the shooter specifically
        double robotAngle = predictedPose.getRotation().getRadians();
        double launcherVx = fieldSpeeds.vxMetersPerSecond + fieldSpeeds.omegaRadiansPerSecond *
                (robotToLauncher.getY() * Math.cos(robotAngle) - robotToLauncher.getX() * Math.sin(robotAngle));
        double launcherVy = fieldSpeeds.vyMetersPerSecond + fieldSpeeds.omegaRadiansPerSecond *
                (robotToLauncher.getX() * Math.cos(robotAngle) - robotToLauncher.getY() * Math.sin(robotAngle));
        Translation2d launcherVel = new Translation2d(launcherVx, launcherVy);

        // C. ITERATIVE CONVERGENCE (The 20-Loop)
        Translation2d launcherPos = predictedPose.transformBy(robotToLauncher).getTranslation();
        double lookaheadDist = goalLocation.getDistance(launcherPos);
        Translation2d virtualGoal = goalLocation;

        for (int i = 0; i < 20; i++) {
            double tof = timeOfFlightMap.get(MathUtil.clamp(lookaheadDist, MIN_DIST, MAX_DIST));
            virtualGoal = goalLocation.minus(launcherVel.times(tof));
            lookaheadDist = goalLocation.getDistance(virtualGoal);
        }
        // Create a Pose2d for visualization
        Pose2d virtualGoalPose = new Pose2d(virtualGoal, new Rotation2d());
        // SmartDashboard.putN("Shooter/VirtualGoal", new Field2d().getRobotPose()); //
        // This is one way, but better:
        SmartDashboard.putNumberArray("Shooter/VirtualGoalPose", new double[] {
                virtualGoal.getX(),
                virtualGoal.getY(),
                0 // Rotation in degrees
        });
        // D. OFFSET SOLVE (Arcsin correction for non-centered shooter)
        Rotation2d fieldToTargetAngle = virtualGoal.minus(predictedPose.getTranslation()).getAngle();
        double sinAngle = MathUtil.clamp(
                robotToLauncher.getTranslation().getY() / virtualGoal.getDistance(predictedPose.getTranslation()), -1.0,
                1.0);
        Rotation2d offsetCorrection = new Rotation2d(Math.asin(sinAngle));

        // Final Heading: Point the launcher at the target
        Rotation2d chassisHeading = fieldToTargetAngle.plus(offsetCorrection).plus(robotToLauncher.getRotation());

        // 5. SELECT RPM BASED ON TARGET
        double finalDist = MathUtil.clamp(shooterPosDistance(predictedPose, virtualGoal), MIN_DIST, MAX_DIST);

        // Choose the map based on the boolean
        double targetRPM = isHubTarget
                ? Math.min(rpmMap.get(finalDist), MAX_RPM)
                : Math.min(stockpileRpmMap.get(finalDist), MAX_RPM);
        return new MovingShotSolution(chassisHeading, targetRPM, Degrees.of(hoodAngleMap.get(finalDist)), finalDist);
    }

    private static double shooterPosDistance(Pose2d pose, Translation2d target) {
        return target.getDistance(pose.transformBy(robotToLauncher).getTranslation());
    }

    // Existing Stationary Solve
    public record ShooterSolution(Angle hoodAngle, double rpm, double physicsRPM, double distanceMeters,
            boolean valid) {
    }

    public static ShooterSolution solve(double distanceMeters) {
        double clamped = MathUtil.clamp(distanceMeters, MIN_DIST, MAX_DIST);
        return new ShooterSolution(Degrees.of(hoodAngleMap.get(clamped)), rpmMap.get(clamped), 0.0, clamped, true);
    }

    public static ShooterSolution fallback() {
        return new ShooterSolution(Degrees.of(65), 3150.0, 0.0, 1.5, true);
    }
}