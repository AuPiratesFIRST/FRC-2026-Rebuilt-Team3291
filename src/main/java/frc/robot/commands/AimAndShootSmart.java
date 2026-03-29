package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lighting.LightingSubsystem;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.Shooter.ShooterAimCalculator.ShooterSolution;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.KickerSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.HubTracker;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
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
    private final LightingSubsystem lighting; // Add this
    private final MedianFilter outlierFilter = new MedianFilter(5);
    private final LinearFilter smoothFilter = LinearFilter.movingAverage(10);
    private double lastSmoothDistance = 2.0;

    public AimAndShootSmart(ShooterSubsystem shooter, HoodSubsystem hood, SwerveSubsystem swerve,
            TurretSubsystem turret, IntakeRollerSubsystem intake, KickerSubsystem kicker, VisionSubsystem vision,
            LightingSubsystem lighting) {
        this.shooter = shooter;
        this.hood = hood;
        this.swerve = swerve;
        this.turret = turret;
        this.intake = intake;
        this.kicker = kicker;
        this.vision = vision;
        this.lighting = lighting;

        // We add Vision to the requirements
        addRequirements(shooter, hood, intake, kicker, vision);
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

        double rawDist = visionDistance.isPresent() && hubIsActive
                ? visionDistance.get()
                : swerve.getPose().getTranslation().getDistance(currentGoal);

        // Apply filters to stop the flywheels from "fighting"
        double medianDist = outlierFilter.calculate(rawDist);
        double finalDistance = smoothFilter.calculate(medianDist);

        // 3. RUN THE SOLVER
        var solution = ShooterAimCalculator.solveMoving(
                swerve.getPose(),
                swerve.getRobotVelocity(),
                swerve.getFieldVelocity(),
                currentGoal,
                hubIsActive);

        // 4. APPLY OUTPUTS
        // We use the solveMoving for the HEADING (the Whip Effect)
        // But we use our FILTERED distance for the RPM
        double correctedRPM = hubIsActive
                ? ShooterAimCalculator.solve(finalDistance).rpm()
                : solution.rpm(); // Use stockpile map if not hub

        // 4. APPLY HARDWARE SETPOINTS
        // Use the solveMoving math for Heading, but use our Vision-Corrected distance
        // for RPM/Hood
        shooter.applyRPM(solution.rpm());
        hood.applyAngle(solution.hoodAngle());

        // 5. THE FIRING HANDSHAKE
        // We only fire if: 1. RPM is stable, 2. We are pointed at the Hub/Goal
        boolean atSpeed = shooter.getActualRPM() >= (correctedRPM * 0.97); // 3% tolerance is better for heavy wheels
        boolean aimed = Math.abs(turret.getHeadingErrorDegrees()) < 1.5; // Tighter tolerance

        // Update the LEDs
        lighting.setAligned(aimed);
        if (atSpeed) {
            intake.setPowerDirect(1.0);
            kicker.setPowerDirect(1.0);
        } else {
            // Pull the ball back slightly so it's not touching the spinning flywheel
            intake.setPowerDirect(1.0);
            kicker.setPowerDirect(-0.1);
        }

        // Debugging
        SmartDashboard.putNumber("Targeting/FinalDistance", finalDistance);
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