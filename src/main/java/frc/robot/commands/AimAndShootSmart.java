package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lighting.LightingSubsystem;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.Shooter.ShooterAimCalculator.ShooterSolution;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.intake.AgitatorSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.KickerSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.HubTracker;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import frc.robot.subsystems.intake.AgitatorSubsystem;

public class AimAndShootSmart extends Command {
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final SwerveSubsystem swerve;
    private final TurretSubsystem turret;
    private final IntakeRollerSubsystem intake;
    private final KickerSubsystem kicker;
    private final VisionSubsystem vision;
    private final LightingSubsystem lighting; // Add this
    private final AgitatorSubsystem agitator;

    private final MedianFilter outlierFilter = new MedianFilter(5);
    private final LinearFilter smoothFilter = LinearFilter.movingAverage(10);
    private double lastSmoothDistance = 2.0;

    public AimAndShootSmart(ShooterSubsystem shooter, HoodSubsystem hood, SwerveSubsystem swerve,
            TurretSubsystem turret, IntakeRollerSubsystem intake, KickerSubsystem kicker, VisionSubsystem vision,
            LightingSubsystem lighting, AgitatorSubsystem agitator) {
        this.shooter = shooter;
        this.hood = hood;
        this.swerve = swerve;
        this.turret = turret;
        this.intake = intake;
        this.kicker = kicker;
        this.vision = vision;
        this.lighting = lighting;
        this.agitator = agitator;

        // We add Vision to the requirements
        addRequirements(shooter, hood, intake, kicker, vision, agitator);
    }

    @Override
    public void initialize() {
        // turret.enableHubTracking();
    }

    public void execute() {
        // 1. Setup Targets
        boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        int[] validTags = isBlue ? VisionConstants.BLUE_HUB_TAGS : VisionConstants.RED_HUB_TAGS;
        Translation2d hubPos = isBlue ? Constants.FieldConstants.HUB_BLUE.toTranslation2d()
                : Constants.FieldConstants.HUB_RED.toTranslation2d();
        Translation2d currentGoal = HubTracker.isActive() ? hubPos
                : (isBlue ? Constants.FieldConstants.STOCKPILE_BLUE : Constants.FieldConstants.STOCKPILE_RED);

        // 2. GET DISTANCE (Vision is prioritized)
        Optional<Double> visionDistance = vision.getDistanceToTagMeters(validTags);
        double odomDist = swerve.getPose().getTranslation().getDistance(currentGoal);

        // Use vision if available, otherwise fallback to odometry
        double rawDist = (visionDistance.isPresent() && HubTracker.isActive()) ? visionDistance.get() : odomDist;

        // Filter the distance to prevent RPM jitter
        double finalDistance = smoothFilter.calculate(outlierFilter.calculate(rawDist));

        // 3. CORRECT THE POSE FOR THE SOLVER
        // This is the "Magic Step". We create a virtual pose that is exactly
        // 'finalDistance' away from the goal along the same line.
        Rotation2d angleToGoal = currentGoal.minus(swerve.getPose().getTranslation()).getAngle();
        Translation2d correctedTranslation = currentGoal.minus(new Translation2d(finalDistance, angleToGoal));
        Pose2d correctedPose = new Pose2d(correctedTranslation, swerve.getPose().getRotation());

        // 4. RUN THE SOLVER
        // We pass the 'correctedPose' so the physics match the vision distance
        var solution = ShooterAimCalculator.solveMoving(
                correctedPose,
                swerve.getRobotVelocity(),
                swerve.getFieldVelocity(),
                currentGoal,
                HubTracker.isActive());

        // 5. APPLY OUTPUTS
        double targetRPM = solution.rpm();
        shooter.applyRPM(targetRPM);
        hood.applyAngle(solution.hoodAngle());

        // 6. THE FIRING HANDSHAKE
        // Use a small deadband for RPM and Turret
        boolean atSpeed = Math.abs(shooter.getActualRPM() - targetRPM) < (targetRPM * 0.03);
        boolean aimed = Math.abs(turret.getHeadingErrorDegrees()) < 2.0;

        lighting.setAligned(aimed);
        if (atSpeed && aimed) {
            intake.setPowerDirect(1.0);
            kicker.setPowerDirect(1.0);
            agitator.setPowerDirect(0.5);
        } else {
            intake.setPowerDirect(0.0);
            kicker.setPowerDirect(-0.1); // Prevent ball from touching wheel
            agitator.setPowerDirect(0.0);
        }

        SmartDashboard.putNumber("Targeting/VisionDist", finalDistance);
        SmartDashboard.putNumber("Targeting/TargetRPM", targetRPM);
        SmartDashboard.putBoolean("Targeting/SystemReady", atSpeed && aimed);

    }

    @Override
    public void end(boolean interrupted) {
        turret.disableHubTracking();
        shooter.idle();
        intake.setPowerDirect(0.0);
        kicker.setPowerDirect(0.0);
    }
}