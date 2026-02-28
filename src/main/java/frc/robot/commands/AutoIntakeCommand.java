package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;

public class AutoIntakeCommand extends Command {

    private final ShooterSubsystem shooter;
    private final IntakeRollerSubsystem intake;
    private final double durationSeconds;
    private final double rollerSpeed;
    private final Timer timer = new Timer();

    public AutoIntakeCommand(ShooterSubsystem shooter, IntakeRollerSubsystem intake, double durationSeconds,
            double rollerSpeed) {
        this.shooter = shooter;
        this.intake = intake;
        this.durationSeconds = durationSeconds;
        this.rollerSpeed = rollerSpeed;

        // CRITICAL: Tell the scheduler this command uses these subsystems.
        // This stops "AimShooterFromVision" automatically when this starts.
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        // Instead of scheduling new commands, call the logic directly.
        // Based on your subsystem methods:
        shooter.applyRPM(-1000); // Or whatever RPM your 'intakeMode' uses
        intake.set(rollerSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop(); // Assuming stop() is a method that sets motor to 0
        intake.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }
}