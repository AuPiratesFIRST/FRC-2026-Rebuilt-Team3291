package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;

public class AutoShootCommand extends Command {
    private final ShooterSubsystem shooter;
    private final IntakeRollerSubsystem intake;

    public AutoShootCommand(
            ShooterSubsystem shooter,
            IntakeRollerSubsystem intake) {
        this.shooter = shooter;
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (shooter.atTargetRPM()) {
            intake.in(1.0).schedule();
        } else {
            intake.out(0.09); // Small reverse speed to prevent jamming while waiting for
                              // flywheel to spin up
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop().schedule();
    }
}