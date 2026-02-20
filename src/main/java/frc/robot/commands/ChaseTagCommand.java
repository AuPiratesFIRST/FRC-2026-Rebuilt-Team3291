package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.TankDrive.Drive;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * ChaseTagCommand - Automatically drives to a position relative to an AprilTag.
 * 
 * This command uses vision to locate an AprilTag, then drives the robot to a
 * specified distance and orientation relative to that tag. Designed for tank drive.
 * 
 * How it works:
 * 1. Vision detects AprilTag and reports its position relative to camera
 * 2. Calculate where the tag is in field coordinates
 * 3. Calculate goal position (where we want robot center to be)
 * 4. Use PID controllers to drive there
 * 
 * Tank Drive Considerations:
 * - Can only move forward/backward and rotate (no strafing)
 * - Uses heading controller to point at goal, then drives forward
 * - Field-relative control converted to robot-relative
 * 
 * Example Usage:
 * - Drive to 3 meters in front of hub AprilTag
 * - Align with charging station tag at specific distance
 */
public class ChaseTagCommand extends Command {

    // Profiled PID controllers limit acceleration for smooth motion
    // X_CONSTRAINTS = [max velocity (m/s), max acceleration (m/s²)]
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2.0, 1.5);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
            Math.PI, // 180°/s
            Math.PI); // 180°/s²

    private final VisionSubsystem vision;
    private final Drive drive;
    private final int[] tagIds;
    private final Transform3d tagToGoal;

    // X controller drives forward/backward to reach goal position
    private final ProfiledPIDController xController = new ProfiledPIDController(3.0, 0, 0.05, X_CONSTRAINTS);
    
    // Omega controller rotates to face the correct heading
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2.5, 0, 0.05,
            OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;
    private Pose2d fieldGoalPose;

    /**
     * Creates a chase tag command for tank drive.
     * 
     * @param vision Vision subsystem to detect tags
     * @param drive Tank drive subsystem
     * @param tagIds Array of valid AprilTag IDs to chase
     * @param targetDistanceMeters How far in front of tag to stop (meters)
     */
    public ChaseTagCommand(
            VisionSubsystem vision,
            Drive drive,
            int[] tagIds,
            double targetDistanceMeters) {
        this.vision = vision;
        this.drive = drive;
        this.tagIds = tagIds;

        // Transform from tag to desired robot position
        // Translation: Move targetDistance forward from tag (X-axis)
        // Rotation: PI radians = 180° means robot faces toward tag
        // (robot's front points at tag)
        this.tagToGoal = new Transform3d(
                new Translation3d(targetDistanceMeters, 0.0, 0.0),
                new Rotation3d(0.0, 0.0, Math.PI));

        // Tolerance = how close is "close enough" to call setpoint reached
        xController.setTolerance(0.08); // 8cm position tolerance
        omegaController.setTolerance(Units.degreesToRadians(3)); // 3° heading tolerance
        omegaController.enableContinuousInput(-Math.PI, Math.PI); // Wrap angles at ±180°

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        fieldGoalPose = null;
        
        // Initialize controllers with current robot state
        var robotPose = drive.getPose();
        xController.reset(robotPose.getX());
        omegaController.reset(robotPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        var robotPose2d = drive.getPose();
        var robotPose3d = new Pose3d(robotPose2d);

        var photonRes = vision.getLatestResult();

        // ============================================================
        // VISION PROCESSING
        // ============================================================
        if (photonRes.hasTargets()) {
            // Filter for valid, unambiguous targets
            var targetOpt = photonRes.getTargets().stream()
                    .filter(t -> isIdValid(t.getFiducialId()))
                    .filter(t -> t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                    .findFirst();

            if (targetOpt.isPresent()) {
                var target = targetOpt.get();
                lastTarget = target;

                // Calculate field positions:
                // 1. Where is camera in field coordinates?
                var cameraPose = robotPose3d.transformBy(VisionConstants.ROBOT_TO_SHOOTER_CAMERA);
                
                // 2. Where is the AprilTag in field coordinates?
                var targetPose = cameraPose.transformBy(target.getBestCameraToTarget());
                
                // 3. Where should robot center be to achieve desired offset from tag?
                fieldGoalPose = targetPose.transformBy(tagToGoal).toPose2d();

                // Update controller goals
                // For tank drive, we need to calculate the distance to goal
                // and the heading to point at the goal
                Translation2d toGoal = fieldGoalPose.getTranslation().minus(robotPose2d.getTranslation());
                double distanceToGoal = toGoal.getNorm();
                
                // Set goal for forward controller (drive toward goal position)
                // We track progress by measuring distance to goal
                xController.setGoal(0); // Goal is to reach zero distance
                
                // Set goal for rotation controller (face the goal pose heading)
                omegaController.setGoal(fieldGoalPose.getRotation().getRadians());
            }
        }

        // ============================================================
        // DRIVE CONTROL
        // ============================================================
        if (lastTarget == null || fieldGoalPose == null) {
            // No valid target - stop
            drive.runClosedLoop(new ChassisSpeeds());
        } else {
            // Calculate how to get to goal
            Translation2d toGoal = fieldGoalPose.getTranslation().minus(robotPose2d.getTranslation());
            double distanceToGoal = toGoal.getNorm();
            
            // Calculate field-relative heading error
            Rotation2d headingToGoal = toGoal.getAngle();
            double headingError = headingToGoal.minus(robotPose2d.getRotation()).getRadians();
            
            // Forward speed: proportional to distance, but adjusted by heading error
            // If we're not facing the goal, slow down forward speed
            double xSpeed = xController.calculate(distanceToGoal);
            
            // Reduce forward speed when not aligned (cos of heading error)
            // When facing goal (error=0): cos(0)=1.0 (full speed)
            // When perpendicular (error=90°): cos(90°)=0.0 (stop)
            xSpeed *= Math.cos(headingError);
            
            // Rotational speed: turn to face goal heading
            double omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());

            // Stop if at goal
            if (xController.atGoal() && Math.abs(distanceToGoal) < 0.1) {
                xSpeed = 0;
            }
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }

            // Tank drive uses robot-relative control
            // ChassisSpeeds(vx, vy, omega) where vy is always 0 for tank
            drive.runClosedLoop(new ChassisSpeeds(xSpeed, 0.0, omegaSpeed));

            // Telemetry for tuning
            SmartDashboard.putNumber("ChaseTag/GoalX", fieldGoalPose.getX());
            SmartDashboard.putNumber("ChaseTag/GoalY", fieldGoalPose.getY());
            SmartDashboard.putNumber("ChaseTag/GoalDeg", fieldGoalPose.getRotation().getDegrees());
            SmartDashboard.putNumber("ChaseTag/DistanceToGoal", distanceToGoal);
            SmartDashboard.putNumber("ChaseTag/HeadingError", Units.radiansToDegrees(headingError));
            SmartDashboard.putBoolean("ChaseTag/AtGoal", isFinished());
        }
    }

    /**
     * Checks if a detected tag ID is in our list of valid targets.
     */
    private boolean isIdValid(int id) {
        for (int validId : tagIds) {
            if (id == validId)
                return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.runClosedLoop(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        if (fieldGoalPose == null) {
            return false;
        }
        
        // Check if we're close enough to goal position
        Translation2d toGoal = fieldGoalPose.getTranslation().minus(drive.getPose().getTranslation());
        double distanceToGoal = toGoal.getNorm();
        
        // Finished when both position and heading are at setpoint
        return distanceToGoal < 0.15 && omegaController.atGoal();
    }
}
