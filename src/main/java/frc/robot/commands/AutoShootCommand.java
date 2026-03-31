package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem; // Your renamed intake file
import frc.robot.subsystems.intake.KickerSubsystem; // Your new router file
import frc.robot.subsystems.intake.AgitatorSubsystem; // Your new agitator file

public class AutoShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final IntakeRollerSubsystem intake;
    private final KickerSubsystem kicker;
    private final AgitatorSubsystem agitator;

    public AutoShootCommand(ShooterSubsystem shooter, IntakeRollerSubsystem intake, KickerSubsystem kicker,
            AgitatorSubsystem agitator) {
        this.shooter = shooter;
        this.intake = intake;
        this.kicker = kicker;
        this.agitator = agitator;

        // We require Intake and Kicker so nothing else can use them while shooting.
        // We do NOT require Shooter, so AimAndShootSmart can keep controlling the RPM!
        addRequirements(intake, kicker);
    }

    @Override
    public void initialize() {
        // Start with a slight reverse to pull the ball away from the flywheel
        intake.setPowerDirect(-0.3);
        kicker.setPowerDirect(-0.3);
        agitator.setPowerDirect(-0.3);
    }

    @Override
    public void execute() {
        // Are we within 5% of our target RPM?
        boolean atSpeed = shooter.getActualRPM() >= (shooter.getTargetRPM() * 0.98);

        if (atSpeed) {
            // FIRE! Push the ball UP through the intake and kicker
            intake.setPowerDirect(1.0);
            kicker.setPowerDirect(0.8);
            agitator.setPowerDirect(0.09);
        } else {
            // HOLD! Wait for RPM to recover.
            // -0.1 holds the ball slightly down so it doesn't rub the flywheel
            intake.setPowerDirect(1);
            kicker.setPowerDirect(-0.4);
            agitator.setPowerDirect(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the feeding motors when we let go of the button
        intake.setPowerDirect(0.0);
        kicker.setPowerDirect(0.0);
        agitator.setPowerDirect(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs as long as the button is held
    }
}