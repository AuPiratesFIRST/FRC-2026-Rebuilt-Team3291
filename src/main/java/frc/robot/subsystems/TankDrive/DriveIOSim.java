// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.TankDrive;

import static frc.robot.subsystems.TankDrive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.imu.GyroIOSim;

/**
 * Simulation implementation of DriveIO using WPILib's physics engine.
 * 
 * This class simulates a tank drivetrain with realistic physics:
 * - Motor torque curves (CIM motors)
 * - Gear reduction and wheel size
 * - Mass, friction, and momentum
 * - Battery voltage sag under load
 * 
 * SIMULATION FEATURES:
 * - Runs entirely on your computer - no robot hardware needed
 * - Faster than real-time (useful for testing many scenarios quickly)
 * - Deterministic (same inputs always produce same outputs)
 * - Updates simulated gyro heading for odometry testing
 * 
 * DIFFERENCES FROM REAL HARDWARE:
 * - Physics model is simplified (no wheel slip, bumps, carpet friction variation)
 * - PID runs on RoboRIO at 50Hz (real Sparks run at 1kHz)
 * - No CAN bus delays or dropouts
 * - No mechanical backlash or wear
 * 
 * This is still incredibly valuable for:
 * - Testing autonomous paths before robot is built
 * - Developing code during build season when robot is unavailable
 * - Reproducing bugs in a controlled environment
 * - Training new programmers without risking real hardware
 */
public class DriveIOSim implements DriveIO {
  // ========================================
  // PHYSICS SIMULATION
  // ========================================
  // WPILib's drivetrain simulator with pre-configured kitbot parameters:
  // - Motor: Dual CIM per side (classic FRC motor choice)
  // - Gearing: 10.71:1 reduction (motor spins 10.71x faster than wheels)
  // - Wheels: 6 inch diameter
  // - Mass: Standard kitbot chassis weight
  private DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

  // ========================================
  // GYRO SIMULATION INTEGRATION
  // ========================================
  // We need to update the simulated gyro with our heading from drivetrain physics
  private final GyroIOSim gyroSim;
  // Track heading changes to calculate angular velocity (gyro rate)
  private Rotation2d lastHeading = new Rotation2d();
  private double lastHeadingTime = Timer.getFPGATimestamp();

  // ========================================
  // CONTROL STATE
  // ========================================
  // Current commanded voltages (what we're sending to simulated motors)
  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  
  // Control mode: false = open-loop (direct voltage), true = closed-loop (PID to velocity)
  private boolean closedLoop = false;
  
  // PID controllers for closed-loop simulation (mimic Spark controller behavior)
  // Real Sparks run PID at 1kHz, but we run at 50Hz on RoboRIO
  private PIDController leftPID = new PIDController(simKp, 0.0, simKd);
  private PIDController rightPID = new PIDController(simKp, 0.0, simKd);
  
  // Feedforward voltages added to PID output in closed-loop mode
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;

  /**
   * Default constructor for simulation without gyro integration.
   * The drivetrain will still track its own heading internally.
   */
  public DriveIOSim() {
    this(null);
  }

  /**
   * Constructor with gyro integration.
   * 
   * When provided, the simulated gyro will be updated with heading and
   * angular velocity from the drivetrain physics. This ensures the IMU
   * subsystem sees realistic gyro data during simulation.
   * 
   * @param gyroSim The simulated gyro to update (or null to skip)
   */
  public DriveIOSim(GyroIOSim gyroSim) {
    this.gyroSim = gyroSim;
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // ========================================
    // STEP 1: CALCULATE VOLTAGES FOR CLOSED-LOOP MODE
    // ========================================
    if (closedLoop) {
      // In closed-loop, we're mimicking the Spark controller's PID behavior
      // PID calculates correction voltage based on error (target velocity - actual velocity)
      // Feedforward voltage is added to help reach target faster
      leftAppliedVolts = leftFFVolts + leftPID.calculate(sim.getLeftVelocityMetersPerSecond() / wheelRadiusMeters);
      rightAppliedVolts = rightFFVolts
          + rightPID.calculate(sim.getRightVelocityMetersPerSecond() / wheelRadiusMeters);
    }
    // In open-loop mode, voltages were already set by setVoltage()

    // ========================================
    // STEP 2: RUN PHYSICS SIMULATION
    // ========================================
    // Apply voltages to simulated motors (clamped to battery voltage limits)
    sim.setInputs(
        MathUtil.clamp(leftAppliedVolts, -12.0, 12.0),
        MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
    // Step simulation forward by 20ms (one loop cycle)
    // Physics engine calculates: motor torque → wheel acceleration → velocity → position
    sim.update(0.02);

    // ========================================
    // STEP 3: UPDATE SIMULATED GYRO (IF CONNECTED)
    // ========================================
    if (gyroSim != null) {
      // Get robot heading from drivetrain physics
      Rotation2d heading = sim.getHeading();
      double now = Timer.getFPGATimestamp();
      double dt = now - lastHeadingTime;
      
      if (dt > 0.0) {
        // Calculate angular velocity (degrees per second) from heading change
        double deltaRad = heading.minus(lastHeading).getRadians();
        // Update simulated gyro with new heading and rate
        gyroSim.setSimYaw(heading);
        gyroSim.setSimYawRate(deltaRad / dt);  // rad/s
      }
      
      // Remember current heading for next cycle's rate calculation
      lastHeading = heading;
      lastHeadingTime = now;
    }

    // ========================================
    // STEP 4: POPULATE SENSOR INPUTS
    // ========================================
    // Convert from linear meters (physics sim units) to radians (encoder units)
    inputs.leftPositionRad = sim.getLeftPositionMeters() / wheelRadiusMeters;
    inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / wheelRadiusMeters;
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] { sim.getLeftCurrentDrawAmps() };

    inputs.rightPositionRad = sim.getRightPositionMeters() / wheelRadiusMeters;
    inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / wheelRadiusMeters;
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] { sim.getRightCurrentDrawAmps() };
  }

  /**
   * Open-loop voltage control.
   * 
   * Switch to open-loop mode and directly set voltages.
   * These voltages will be applied in the next updateInputs() call.
   */
  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;  // Disable PID control
    leftAppliedVolts = leftVolts;
    rightAppliedVolts = rightVolts;
  }

  /**
   * Closed-loop velocity control.
   * 
   * Switch to closed-loop mode and set velocity targets.
   * PID controllers will calculate voltages in updateInputs() to reach targets.
   */
  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;  // Enable PID control
    // Store feedforward voltages (will be added to PID output)
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
    // Set PID target velocities
    leftPID.setSetpoint(leftRadPerSec);
    rightPID.setSetpoint(rightRadPerSec);
  }
}
