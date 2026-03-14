package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.KickerSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.HubTracker;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class AimAndShootSmart extends Command {
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final SwerveSubsystem swerve;
    private final TurretSubsystem turret;
    private final IntakeRollerSubsystem intake;
    private final KickerSubsystem kicker;
    private final VisionSubsystem vision;

    public AimAndShootSmart(ShooterSubsystem shooter, HoodSubsystem hood, SwerveSubsystem swerve,
            TurretSubsystem turret, IntakeRollerSubsystem intake, KickerSubsystem kicker, VisionSubsystem vision) {
        this.shooter = shooter;
        this.hood = hood;
        this.swerve = swerve;
        this.turret = turret;
        this.intake = intake;
        this.kicker = kicker;
        this.vision = vision;

        // We add Vision to the requirements
        addRequirements(shooter, hood, intake, kicker);
    }

    @Override
    public void initialize() {
        turret.enableHubTracking();
    }

    @Override
    public void execute() {
        // 1. Setup Alliance and Targets
        boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        int[] validTags = isBlue ? VisionConstants.BLUE_HUB_TAGS : VisionConstants.RED_HUB_TAGS;

        Translation2d hubPos = isBlue ? Constants.FieldConstants.HUB_BLUE.toTranslation2d()
                : Constants.FieldConstants.HUB_RED.toTranslation2d();
        Translation2d stockpilePos = isBlue ? Constants.FieldConstants.STOCKPILE_BLUE
                : Constants.FieldConstants.STOCKPILE_RED;

        boolean hubIsActive = HubTracker.isActive();
        Translation2d currentGoal = hubIsActive ? hubPos : stockpilePos;

        // 2. VISION VS ODOMETRY DISTANCE
        // We get the raw vision distance if available
        Optional<Double> visionDistance = vision.getDistanceToTagMeters(validTags);
        double finalDistance;

        if (visionDistance.isPresent() && hubIsActive) {
            // Use high-precision Vision Distance
            finalDistance = visionDistance.get();
            SmartDashboard.putString("Targeting/Source", "VISION");
        } else {
            // Fallback to Odometry Distance (or use it for Stockpiling)
            finalDistance = swerve.getPose().getTranslation().getDistance(currentGoal);
            SmartDashboard.putString("Targeting/Source", "ODOMETRY");
        }

        // 3. RUN THE DYNAMIC SOLVER
        // We still use the solveMoving logic to handle the robot's velocity (Whip
        // Effect)
        var solution = ShooterAimCalculator.solveMoving(
                swerve.getPose(),
                swerve.getRobotVelocity(),
                swerve.getFieldVelocity(),
                currentGoal, hubIsActive);

        // 4. APPLY HARDWARE SETPOINTS
        // Use the solveMoving math for Heading, but use our Vision-Corrected distance
        // for RPM/Hood
        shooter.applyRPM(solution.rpm());
        hood.applyAngle(solution.hoodAngle());

        // 5. THE FIRING HANDSHAKE
        // We only fire if: 1. RPM is stable, 2. We are pointed at the Hub/Goal
        boolean atSpeed = shooter.getActualRPM() >= (solution.rpm() * 0.97);
        boolean aimed = Math.abs(swerve.getPose().getRotation().minus(solution.chassisHeading()).getDegrees()) < 2.0;

        if (atSpeed && aimed) {
            intake.setPowerDirect(1.0);
            kicker.setPowerDirect(1.0);
        } else {
            // Pull the ball back slightly so it's not touching the spinning flywheel
            intake.setPowerDirect(-0.1);
            kicker.setPowerDirect(-0.1);
        }

        // Debugging
        SmartDashboard.putNumber("Targeting/FinalDistance", finalDistance);
        SmartDashboard.putBoolean("Targeting/SystemReady", atSpeed && aimed);
    }

    @Override
    public void end(boolean interrupted) {
        turret.disableHubTracking();
        shooter.stop();
        intake.setPowerDirect(0.0);
        kicker.setPowerDirect(0.0);
    }
}