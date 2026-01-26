package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.Constants.VisionConstants;

import java.util.Optional;

public class AimShooterFromVision extends Command {

    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final VisionSubsystem vision;

    public AimShooterFromVision(
        ShooterSubsystem shooter,
        HoodSubsystem hood,
        VisionSubsystem vision
    ) {
        this.shooter = shooter;
        this.hood = hood;
        this.vision = vision;

        addRequirements(shooter, hood);
    }

    @Override
    public void execute() {

        Optional<Double> distance =
            vision.getDistanceToTagMeters(
                VisionConstants.BLUE_HUB_TAGS
            );

        if (distance.isEmpty()) {
            shooter.applyRPM(0);
            return;
        }

        var solution =
            ShooterAimCalculator.solve(distance.get());

        if (!solution.valid()) {
            shooter.applyRPM(0);
            return;
        }

        hood.applyAngle(solution.hoodAngle());
        shooter.applyRPM(solution.rpm());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.applyRPM(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
