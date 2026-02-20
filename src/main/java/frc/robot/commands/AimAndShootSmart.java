package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.Turret.TurretSubsystem;

/**
 * Unified Scoring Command
 * Uses the advanced "Shoot on the Move" math hidden in the TurretSubsystem,
 * but follows the clean lifecycle of a standard command.
 */
public class AimAndShootSmart extends Command {

    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;

    public AimAndShootSmart(TurretSubsystem turret, ShooterSubsystem shooter, HoodSubsystem hood) {
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        // We require these so other commands don't move the shooter/hood while aiming
        addRequirements(turret, shooter, hood);
    }

    @Override
    public void initialize() {
        // Turn on the advanced moving-target math in the subsystem
        turret.enableHubTracking();
    }

    @Override
    public void execute() {
        // The TurretSubsystem.periodic() is already doing the:
        // 1. Latency compensation
        // 2. Vector subtraction
        // 3. Distance-to-RPM lookups

        // We just need to check if we are on target and "Ready to Fire"
        boolean aimingReady = Math
                .abs(turret.getDesiredRobotHeading().minus(turret.getDesiredRobotHeading()).getDegrees()) < 2.0;
        boolean shooterReady = Math.abs(shooter.getActualRPM() - shooter.getTargetRPM()) < 50;

        if (aimingReady && shooterReady) {
            // Optional: You could trigger the intake/feeder here automatically
            // feeder.run();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.disableHubTracking();
        shooter.stop().schedule();
    }

    @Override
    public boolean isFinished() {
        // Typically false for teleop (held button),
        // but could be true in Auto after 1 shot.
        return false;
    }
}