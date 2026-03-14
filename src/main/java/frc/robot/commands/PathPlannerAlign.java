package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class PathPlannerAlign extends Command {
    private final TurretSubsystem turret;
    private final SwerveSubsystem swerve;

    public PathPlannerAlign(TurretSubsystem turret, SwerveSubsystem swerve) {
        this.turret = turret;
        this.swerve = swerve;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.enableHubTracking();
        // Tell PathPlanner to use our rotation speed while driving
        PPHolonomicDriveController.overrideRotationFeedback(turret::getDesiredRobotOmega);
    }

    @Override
    public void execute() {
        // Only manually drive the wheels if PathPlanner IS NOT currently driving a
        // path.
        // We detect this by checking if the swerve's current command is us.
        if (swerve.getCurrentCommand() == this) {
            double omega = turret.getDesiredRobotOmega();
            swerve.drive(new ChassisSpeeds(0, 0, omega));
        }
    }

    @Override
    public boolean isFinished() {
        // If this is a Marker inside a path, we don't want it to finish early.
        // If this is a standalone command in an Auto, we want it to finish when aimed.
        if (swerve.getCurrentCommand() == this) {
            // Finish if error is less than 2 degrees
            return Math.abs(turret.getHeadingErrorDegrees()) < 2.0;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // We do NOT call disableHubTracking here if we want to stay aimed for shooting.
        // But we MUST clear the override.
        PPHolonomicDriveController.clearRotationFeedbackOverride();
        swerve.drive(new ChassisSpeeds(0, 0, 0));
    }
}