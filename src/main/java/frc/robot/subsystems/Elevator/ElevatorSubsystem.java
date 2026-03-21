package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ElevatorSubsystem extends SubsystemBase {

    // ---------------- HARDWARE ----------------
    private final SparkMax motor = new SparkMax(28, MotorType.kBrushless);

    // ---------------- CONFIG ----------------
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withMechanismCircumference(Meters.of(0.05))
            .withClosedLoopController(
                    5.9, 0, 0.12, // Tune P higher if it can't fight the springs
                    MetersPerSecond.of(1.5),
                    MetersPerSecondPerSecond.of(3.0))
            /*
             * NOTE: If springs pull the elevator UP, the Feedforward 'kg'
             * might need to be a negative value to provide a constant "pull down" force.
             */
            .withFeedforward(new ElevatorFeedforward(0.1, -0.5, 0))
            .withTelemetry("Elevator/Motor", TelemetryVerbosity.LOW)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(180, 1)))
            .withMotorInverted(false)
            .withStatorCurrentLimit(Amps.of(50)) // Increased slightly to handle holding tension
            .withIdleMode(MotorMode.BRAKE);

    private final SmartMotorController smartMotor = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

    // ---------------- MECHANISM ----------------
    private final ElevatorConfig elevatorConfig = new ElevatorConfig(smartMotor)
            .withStartingHeight(Meters.of(0))
            .withHardLimits(Meters.of(0), Meters.of(1.5))
            .withMass(Kilograms.of(5))
            .withTelemetry("Elevator/Mech", TelemetryVerbosity.LOW);

    private final Elevator elevator = new Elevator(elevatorConfig);

    // ---------------- CONSTRUCTOR ----------------
    public ElevatorSubsystem() {
        // This ensures that as soon as the robot enables,
        // the motor pulls the elevator to 0 and HOLDS it there.
    }

    // ---------------- COMMAND FACTORIES ----------------

    /** The default state: Pull down to 0 meters and hold against springs */
    public Command stow() {
        return elevator.run(Meters.of(0)).withName("ElevatorStow");
    }

    /** Runs elevator continuously to a specific height */
    public Command setHeight(Distance height) {
        return elevator.run(height);
    }

    /** Goes to height and finishes when it arrives */
    public Command goToHeight(Distance height) {
        return elevator.runTo(height, Meters.of(0.02));
    }

    /** Manual control (Duty Cycle) */
    public Command manual(double percent) {
        return elevator.set(percent);
    }

    /** SysId characterization */
    public Command sysId() {
        return elevator.sysId(
                Volts.of(6),
                Volts.of(2).per(Second),
                Seconds.of(4));
    }

    // ---------------- PERIODIC ----------------

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        elevator.simIterate();
    }
}