package frc.robot.subsystems.Lighting;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Lighting.Colors;

public class LightingSubsystem extends SubsystemBase {
    private final Spark lighting;
    // private final SendableChooser<Colors> lighting_chooser = new
    // SendableChooser<>();

    // State Variables
    private boolean motorIsHot = false;
    private boolean isAligned = false;
    private boolean hasNote = false;

    public LightingSubsystem() {
        lighting = new Spark(Constants.Lighting.lightingPort);
        // ... (keep your chooser logic here)
    }

    // Methods for other subsystems to call
    public void setMotorHot(boolean hot) {
        this.motorIsHot = hot;
    }

    public void setAligned(boolean aligned) {
        this.isAligned = aligned;
    }

    public void setHasNote(boolean note) {
        this.hasNote = note;
    }

    @Override
    public void periodic() {
        // PRIORITY LOGIC
        if (motorIsHot) {
            // MOST IMPORTANT: BLINK RED if any motor is over 50 degrees
            lighting.set(Colors.BLINK_RED.getColorValue());
        } else if (isAligned) {
            // SECOND: Solid Lime/Green if we are pointed at the target
            lighting.set(Colors.LIME.getColorValue());
        } else if (hasNote) {
            // THIRD: Orange if a note is in the intake
            lighting.set(Colors.ORANGE.getColorValue());
        } else {
            // DEFAULT: Alliance color from the chooser
            // lighting.set(lighting_chooser.getSelected().getColorValue());
        }
    }
}