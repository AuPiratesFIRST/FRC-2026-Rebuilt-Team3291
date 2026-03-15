package frc.robot.subsystems.Lighting;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Lighting.Colors;

public class LightingSubsystem extends SubsystemBase {

    public Spark lighting;

    public final SendableChooser<Constants.Lighting.Colors> lighting_chooser = new SendableChooser<>();

    /**
     * Creates a new LightingSubsys
     * tem.
     */
    public LightingSubsystem() {
        lighting = new Spark(Constants.Lighting.lightingPort);

        lighting.set(Constants.Lighting.startingColor.getColorValue());

        lighting_chooser.setDefaultOption(Constants.Lighting.startingColor.getColorName(),
                Constants.Lighting.startingColor);

        for (Colors c : Colors.values()) {
            lighting_chooser.addOption(c.getColorName(), c);
        }

        SmartDashboard.putData("Alliance", lighting_chooser);
    }

    public void blink() {
        Colors selectedColor = lighting_chooser.getSelected();

        lighting.set(selectedColor.getColorValue());
        lighting.set(0.99);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Colors selectedColor = lighting_chooser.getSelected();

        lighting.set(selectedColor.getColorValue());
    }

}
