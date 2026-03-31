package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.LinearVelocity;

import org.littletonrobotics.junction.Logger;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.*;
import yams.motorcontrollers.local.SparkWrapper;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Shooter Subsystem - Controls the flywheel that launches game pieces.
 * 
 * This subsystem manages a single NEO motor spinning a flywheel wheel to shoot
 * game pieces.
 * The flywheel speed is controlled in RPM (revolutions per minute).
 * 
 * Key Features:
 * - Closed-loop velocity control using YAMS (Yet Another Motor library)
 * - Configurable RPM setpoints for different shot distances
 * - Geared 3:4 reduction for higher torque
 * - Coast mode when disabled to reduce motor heating
 * - Full physics simulation support
 * 
 * The shooter uses a command factory pattern - call setRPM() to get a command
 * that will spin up the flywheel and maintain the speed.
 */
public class ShooterSubsystem extends SubsystemBase {

    // Maximum safe RPM for the flywheel (limited by motor and mechanical
    // constraints)
    private static final double MAX_RPM = 6000.0;
    // A good starting point for KitBot intake speed is 1000-1500 RPM
    private static final double INTAKE_RPM = 3900.0;
    private static final double OUTTAKE_RPM = -3900.0;

    // Default idle to keep belts moving and overcome static friction
    private static final double IDLE_RPM = 260.0;

    // ========== HARDWARE ==========
    // SparkMax motor controller controlling one NEO brushless motor
    // CAN ID 28 must match what's configured in REV Hardware Client
    private final SparkMax shooterMotor = new SparkMax(14, MotorType.kBrushless);

    // ========== YAMS SMART MOTOR CONTROLLER ==========
    // YAMS wraps the SparkMax and adds features like automatic logging,
    // simulation, and simplified configuration
    private final SmartMotorController shooterSMC = new SparkWrapper(
            shooterMotor,
            DCMotor.getNEO(1), // Motor model for physics simulation
            new SmartMotorControllerConfig(this)
                    // Use closed-loop (PID) control for accurate velocity
                    .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
                    // PID gains: P=0.001 (gentle), I=0, D=0
                    // Max velocity = 6000 RPM, max acceleration = 600 RPM/s
                    .withClosedLoopController(
                            0.02654, 0, 0.,
                            RPM.of(MAX_RPM),
                            RotationsPerSecondPerSecond.of(3600))
                    // Feedforward: kS=0.25V, kV=0.12V/(rad/s), kA=0.015V/(rad/s²)
                    .withFeedforward(
                            new SimpleMotorFeedforward(
                                    0.165, 0.1199, 1.7))

                    .withSimFeedforward(new SimpleMotorFeedforward(0.25, 0.12, 0.015))

                    // 1:1 gear reduction (motor spins faster than flywheel)
                    .withGearing(
                            new MechanismGearing(
                                    GearBox.fromReductionStages(1, 1)))
                    // Coast mode = motor freewheels when disabled (reduces heat)
                    .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
                    // Limit current to 50A to prevent brownouts
                    .withStatorCurrentLimit(Amps.of(40))
                    .withMotorInverted(true)
                    // Medium verbosity logging
                    .withTelemetry("ShooterMotor",
                            SmartMotorControllerConfig.TelemetryVerbosity.LOW));

    private final FlyWheel flywheel = new FlyWheel(
            new FlyWheelConfig(shooterSMC)
                    .withDiameter(Inches.of(4))
                    .withMass(Pounds.of(3.58))
                    .withUpperSoftLimit(RPM.of(MAX_RPM))
                    .withTelemetry("ShooterMech",
                            SmartMotorControllerConfig.TelemetryVerbosity.LOW));

    private double lastTargetRPM = 0.0;

    // ------------------------------------------------
    // COMMAND FACTORIES
    // ------------------------------------------------

    public Command setRPM(double rpm) {
        return flywheel.setSpeed(() -> {
            lastTargetRPM = rpm;
            return RPM.of(rpm);
        });
    }

    public Command stop() {
        return setRPM(0);
    }

    public Command idle() {
        return setRPM(IDLE_RPM).withName("ShooterIdle");
    }

    // Mutable holders for logging (prevents memory allocation every loop)
    private final MutVoltage m_appliedVoltage = new MutVoltage(0, 0, Volts);
    private final MutAngle m_distance = new MutAngle(0, 0, Rotations);
    private final MutAngularVelocity m_velocity = new MutAngularVelocity(0, 0, RotationsPerSecond);

    // Define the Routine manually so we can call .quasistatic() and .dynamic()
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.5).per(Second), // Ramp rate
                    Volts.of(7), // Step voltage
                    Seconds.of(10) // Timeout
            ),
            new SysIdRoutine.Mechanism(
                    (volts) -> shooterSMC.setDutyCycle(volts.in(Volts) / RobotController.getBatteryVoltage()),
                    log -> {
                        shooterSMC.updateTelemetry();
                        shooterSMC.simIterate();
                        log.motor("Shooter")
                                .voltage(m_appliedVoltage.mut_replace(
                                        shooterSMC.getDutyCycle() * RobotController.getBatteryVoltage(), Volts))
                                .angularPosition(m_distance.mut_replace(shooterSMC.getMechanismPosition()))
                                .angularVelocity(m_velocity.mut_replace(shooterSMC.getMechanismVelocity()));
                    },
                    this,
                    "Shooter"));

    /**
     * Runs the full SysId sequence: Quasistatic Forward -> Wait -> Dynamic Forward.
     */
    public Command getSysIdCommand() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                .until(() -> getActualRPM() > MAX_RPM) // Safety check
                .andThen(Commands.runOnce(() -> shooterSMC.setDutyCycle(0))) // Stop motor
                .andThen(Commands.waitSeconds(5.0)) // Wait for 3.08lb mass to stop fully
                .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                        .until(() -> getActualRPM() > MAX_RPM))
                .andThen(Commands.runOnce(() -> shooterSMC.setDutyCycle(0)))
                .withName("ShooterSysId");
    }

    // ------------------------------------------------
    // DIRECT APPLY (USED BY COMMANDS)
    // ------------------------------------------------
    public LinearVelocity getExitVelocity() {
        // Flywheel diameter = 4 inches
        double diameterMeters = Inches.of(4).in(Meters);
        double circumference = Math.PI * diameterMeters;

        // RPM → rotations per second
        double rps = flywheel.getSpeed().in(RPM) / 60.0;

        // v = rps × circumference
        return MetersPerSecond.of(rps * circumference);
    }

    public void applyRPM(double rpm) {
        lastTargetRPM = rpm;
        flywheel.setMechanismVelocitySetpoint(RPM.of(rpm));
    }

    public double getTargetRPM() {
        return lastTargetRPM;
    }

    public double getTemperature() {
        // This returns the temperature in Celsius
        return shooterMotor.getMotorTemperature();
    }

    public double getActualRPM() {
        return flywheel.getSpeed().in(RPM);
    }

    public boolean isAtTarget() {
        // Standard 5% tolerance check
        return Math.abs(getActualRPM() - getTargetRPM()) < (getTargetRPM() * 0.05);
    }

    /**
     * Command to run the motor at a safe speed for intaking.
     * This moves the belts enough to pull balls in, but doesn't launch them.
     */
    public Command intakeMode() {
        return setRPM(INTAKE_RPM).withName("ShooterIntakeMode");
    }

    /**
     * Command to run the motor at a safe speed for outtaking.
     * This moves the belts enough to eject balls, but doesn't launch them.
     */
    public Command outtakeMode() {
        return setRPM(OUTTAKE_RPM).withName("ShooterOuttakeMode");
    }

    // ------------------------------------------------
    // PERIODIC
    // ------------------------------------------------

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/TargetRPM", lastTargetRPM);
        Logger.recordOutput("Shooter/ActualRPM",
                flywheel.getSpeed().in(RPM));

        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}