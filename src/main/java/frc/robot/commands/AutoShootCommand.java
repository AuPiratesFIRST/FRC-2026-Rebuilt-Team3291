package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;

public class AutoShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final IntakeRollerSubsystem intake;

    // Track current state so we don’t reschedule every 20ms
    private boolean feeding = false;

    public AutoShootCommand(
            ShooterSubsystem shooter,
            IntakeRollerSubsystem intake) {

        this.shooter = shooter;
        this.intake = intake;

        // DO NOT addRequirements(intake)
        // Intake commands themselves will own the subsystem
    }

    @Override
    public void initialize() {
        // Start with slight reverse to prevent jams
        intake.out(0.2).schedule();
        feeding = false;
    }

    @Override
    public void execute() {

        boolean atSpeed = shooter.getActualRPM() >= shooter.getTargetRPM() * 0.95;

        // Transition: NOT feeding -> feeding
        if (atSpeed && !feeding) {
            intake.in(1.0).schedule();
            feeding = true;
        }

        // Transition: feeding -> waiting
        if (!atSpeed && feeding) {
            intake.out(0.09).schedule();
            feeding = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop().schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}