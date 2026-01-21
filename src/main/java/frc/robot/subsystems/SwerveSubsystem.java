package frc.robot.subsystems;

// These "Imports" are just grabbing tools from the library so we don't have to build them ourselves.
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * THE SWERVE SUBSYSTEM
 * 
 * Think of this class as the "Software Driver" for your hardware. 
 * It sits between the Joystick (controller) and the physical Motors.
 * 
 * Its job is to read the JSON configuration files and tell the YAGSL library:
 * "Here is what hardware I have, please make it drive."
 */
public class SwerveSubsystem extends SubsystemBase {

  /**
   * The SwerveDrive object is the "Brain" of the library.
   * It handles all the complex math (Trigonometry) to figure out which way
   * to point wheels and how fast to spin them.
   */
  private final SwerveDrive swerveDrive;

  /**
   * --- TUNING VALUE: MAXIMUM SPEED ---
   * 
   * This determines how fast the robot moves when the joystick is pushed 100%.
   * Unit: Meters Per Second (m/s).
   * 
   * - 4.5 m/s is very fast (Good for competition).
   * - 1.0 m/s is very slow (Good for testing safely in a small room).
   * 
   * IF THE ROBOT IS TOO FAST: Change 4.5 to a smaller number (like 2.0).
   */
  double maximumSpeed = 4.5;

  /**
   * CONSTRUCTOR
   * This code runs exactly ONE time when the robot turns on.
   * It loads the settings and prepares the motors.
   * 
   * @param directory The folder on the RoboRIO where your JSON files are stored.
   */
  public SwerveSubsystem(File directory) {
    
    // Telemetry setting:
    // HIGH = Sends a lot of data to the dashboard (Wheel speeds, angles, errors).
    //        Useful for fixing problems.
    // LOW  = Sends less data. Useful for matches to save computer power.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      // 1. Read the JSON files.
      // 2. Build the "SwerveDrive" object.
      // 3. Tell it our maximum speed is 4.5 m/s.
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);

    } catch (Exception e) {
      // If this error happens, it means the code couldn't find your "deploy/swerve" folder.
      // The robot code will crash here to alert you to fix the file path.
      throw new RuntimeException("Error loading Swerve Configuration", e);
    }
  }

  /**
   * THE DRIVE COMMAND
   * This method runs continuously (every 20 milliseconds) while you are driving.
   * It connects the Joystick inputs to the Motor outputs.
   *
   * @param translationX How much we want to move Forward/Backward.
   * @param translationY How much we want to move Left/Right.
   * @param angularRotation How much we want to Spin.
   * @return A Command that the RobotContainer can run.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotation) {
    // "run" starts the loop that repeats 50 times a second.
    return run(() -> {
      
      // --- LOGIC: SPEED SCALING ---
      // The Joystick gives us a number between -1.0 and 1.0 (Percentage).
      // The Robot expects a speed in Meters Per Second.
      // So we do math: (Joystick Percentage) * (Max Speed) = (Target Speed).
      
      // Example: 
      // Joystick pushed halfway (0.5) * Max Speed (4.5) = Robot drives at 2.25 m/s.
      double xVelocity   = translationX.getAsDouble() * maximumSpeed;
      double yVelocity   = translationY.getAsDouble() * maximumSpeed;
      
      // For Rotation (Spinning), we act similarly.
      // Here, 4.5 represents "Radians per Second". 
      // 6.28 radians is a full circle. So 4.5 is about 3/4 of a spin per second.
      double angVelocity = angularRotation.getAsDouble() * maximumSpeed;

      // --- SENDING COMMANDS TO MOTORS ---
      swerveDrive.drive(
          // 1. The X and Y speed combined into a "Translation Vector"
          new Translation2d(xVelocity, yVelocity),
          
          // 2. The Spin Speed
          angVelocity,
          
          // 3. Field Relative (True/False)
          // TRUE = "Forward" on the stick always goes down the field (away from driver).
          //        This is how Swerve is usually driven.
          // FALSE = "Forward" on the stick goes whichever way the robot nose is pointing.
          //         (Like driving a car).
          true,
          
          // 4. Open Loop (True/False)
          // FALSE = The computer checks the speedometer and adjusts power to hit exact speed.
          //         (Standard for driving).
          // TRUE  = The computer just sends raw voltage without checking speed.
          false
      );
    });
  }

  /**
   * Helper: Get Position
   * Required for Autonomous.
   * This asks the robot: "Where are you on the field right now?"
   * @return X, Y, and Angle.
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Helper: Zero Gyro
   * This is the "Reset" button. 
   * If the robot thinks "Forward" is sideways, the driver presses 'A',
   * and this code sets the current direction to be the new "Forward".
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Periodic Loop
   * This runs in the background automatically.
   * YAGSL uses this to update the Odometer (where the robot is) behind the scenes.
   */
  @Override
  public void periodic() {
    // No extra code needed here, YAGSL handles it.
  }
}