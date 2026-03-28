package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.KickerSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterAimCalculator;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutoShootOnTheMove extends Command {
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final SwerveSubsystem swerve;
    private final TurretSubsystem turret;
    private final IntakeRollerSubsystem intake;
    private final KickerSubsystem kicker;
    private final VisionSubsystem vision;

    private final Timer finishTimer = new Timer();
    private boolean hasFired = false;

    public AutoShootOnTheMove(
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            SwerveSubsystem swerve,
            TurretSubsystem turret,
            IntakeRollerSubsystem intake,
            KickerSubsystem kicker,
            VisionSubsystem vision) {

        this.shooter = shooter;
        this.hood = hood;
        this.swerve = swerve;
        this.turret = turret;
        this.intake = intake;
        this.kicker = kicker;
        this.vision = vision;

        // We require the scoring components.
        // We still do NOT require Swerve so PathPlanner doesn't stop.
        addRequirements(shooter, hood, turret, intake, kicker);
    }

    @Override
    public void initialize() {
        finishTimer.reset();
        hasFired = false;

        // 1. Start Hub Tracking
        turret.enableHubTracking();

        // 2. TELL PATHPLANNER TO USE OUR ROTATION WHILE DRIVING
        PPHolonomicDriveController.overrideRotationFeedback(turret::getDesiredRobotOmega);
    }

    @Override
    public void execute() {
        // --- ALIGNMENT LOGIC ---
        // If this command is the "Primary" command on Swerve (meaning NO path is
        // running),
        // we must manually drive the rotation.
        if (swerve.getCurrentCommand() == this) {
            double omega = turret.getDesiredRobotOmega();
            swerve.drive(new ChassisSpeeds(0, 0, omega));
        }

        // --- SHOOTER PHYSICS LOGIC ---
        boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        Translation2d hubPos = isBlue ? Constants.FieldConstants.HUB_BLUE.toTranslation2d()
                : Constants.FieldConstants.HUB_RED.toTranslation2d();

        // Calculate physics (Moving shot compensation)
        var solution = ShooterAimCalculator.solveMoving(
                swerve.getPose(),
                swerve.getRobotVelocity(),
                swerve.getFieldVelocity(),
                hubPos,
                true);

        // Apply hardware setpoints
        shooter.applyRPM(solution.rpm());
        hood.applyAngle(solution.hoodAngle());

        // --- FIRING LOGIC ---
        boolean atSpeed = shooter.getActualRPM() >= (solution.rpm() * 0.98);
        boolean aimed = Math.abs(turret.getHeadingErrorDegrees()) < 2.0;
        turret.shoot();
        if (atSpeed && aimed) {

            intake.setPowerDirect(1.0);
            kicker.setPowerDirect(1.0);
            hasFired = true;
            finishTimer.start();
        } else {
            intake.setPowerDirect(-0.1);
            kicker.setPowerDirect(-0.1);
        }
    }

    @Override
    public boolean isFinished() {
        // If we are a Standalone command (not in a path), finish when fired.
        // If we are a Marker in a path, finish when fired.
        return hasFired && finishTimer.hasElapsed(0.65);
    }

    @Override
    public void end(boolean interrupted) {
        PPHolonomicDriveController.clearRotationFeedbackOverride();

        shooter.idle();
        intake.setPowerDirect(0.0);
        kicker.setPowerDirect(0.0);

        // Only stop the wheels if we were the ones driving them
        if (swerve.getCurrentCommand() == this) {
            swerve.drive(new ChassisSpeeds(0, 0, 0));
        }
    }
}