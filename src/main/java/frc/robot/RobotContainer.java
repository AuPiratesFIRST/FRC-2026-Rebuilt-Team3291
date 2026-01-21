// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * THE ROBOT CONTAINER
 * 
 * Think of this class as the "Pilot's Cockpit" or the "Control Room".
 * 
 * It has three main jobs:
 * 1. It sets up the Subsystems (The robot's physical hardware like the Drive Base).
 * 2. It sets up the Controllers (The Joysticks).
 * 3. It "wires" the buttons on the controller to specific actions on the robot.
 */
public class RobotContainer {

  /**
   * 1. CREATE THE DRIVE BASE
   * 
   * Here we initialize the "SwerveSubsystem". This represents the 4 wheels and motors.
   * 
   * "Filesystem.getDeployDirectory()" is a special command that automatically finds 
   * the correct folder on the RoboRIO, no matter which laptop uploaded the code.
   * 
   * We tell it to look inside the "swerve" folder for our JSON configuration files.
   */
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve")
  );

  /**
   * 2. DEFINE THE CONTROLLER
   * 
   * We are defining the Driver's Controller here.
   * 
   * Port 0: This usually refers to the first USB controller plugged into the laptop.
   * "CommandXboxController": We use this type because the Logitech F310 (in 'X' mode)
   * behaves exactly like an Xbox controller, which makes programming it very easy.
   */
  private final CommandXboxController driverController = new CommandXboxController(0);

  /**
   * CONSTRUCTOR
   * This code runs exactly once when the robot turns on.
   * It calls the method below to "wire up" the buttons.
   */
  public RobotContainer() {
    configureBindings();
  }

  /**
   * CONFIGURE BUTTON BINDINGS
   * This is where we tell the robot what to do when buttons are pressed or sticks are moved.
   */
  private void configureBindings() {

    /**
     * 3. SETTING THE DEFAULT DRIVE COMMAND
     * 
     * "Default Command" means: "If the driver isn't pressing any other special buttons,
     * just keep doing this." In this case, it means "Listen to the joysticks and drive."
     */
    drivebase.setDefaultCommand(
        drivebase.driveCommand(
            
            // --- LEFT STICK Y: Moving Forward & Backward ---
            // "DoubleSupplier" (() -> ...) means we check this value continuously, 50 times a second.
            
            // LOGIC EXPLANATION:
            // 1. driverController.getLeftY(): Reads the stick.
            // 2. MathUtil.applyDeadband(..., 0.1): This is a "Safety Zone". If the stick is 
            //    drifting slightly (less than 10%), we ignore it so the robot doesn't creep away.
            // 3. The Negative Sign (-): This is crucial! On joysticks, pushing UP gives a 
            //    Negative number (-1). But we want the robot to go FORWARD with Positive speed.
            //    So we multiply by -1 to flip it.
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
            
            // --- LEFT STICK X: Moving Left & Right (Strafing) ---
            // Same logic as above.
            // 1. Get the value.
            // 2. Ignore small drift (Deadband).
            // 3. Invert it (-) because pushing LEFT gives a negative number, but we want 
            //    positive movement on the Y-axis (Left).
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
            
            // --- RIGHT STICK X: Spinning (Rotation) ---
            // This controls how fast the robot spins in place.
            // We invert it (-) because usually pushing Left spins Counter-Clockwise, 
            // which math considers "Positive" rotation.
            () -> -driverController.getRightX()
        )
    );

    /**
     * 4. THE "RESET GYRO" BUTTON (Button A)
     * 
     * Sometimes, the robot's internal compass (Gyro) gets confused about where "North" or "Forward" is.
     * This usually happens if the robot spins a lot or gets hit.
     * 
     * When the driver presses 'A', we tell the robot: "Reset! The direction you are facing RIGHT NOW is Forward."
     */
    driverController.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
  }
  
  /**
   * AUTONOMOUS COMMAND
   * This method is called to get the command that runs in Autonomous mode.
   * Right now, it returns "No command configured," so the robot will sit still.
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}