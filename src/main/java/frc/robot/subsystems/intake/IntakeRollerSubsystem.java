package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {

    // Approx inertia for an intake roller (kg·m²)
    private static final double kRollerMOI = 0.00032;

    private final SparkMax rollerMotor = new SparkMax(30, MotorType.kBrushless);

    private final DCMotor rollerMotorModel = DCMotor.getNeoVortex(1);

    private final FlywheelSim rollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    rollerMotorModel,
                    kRollerMOI,
                    1.0),
            rollerMotorModel,
            1.0 / 4096.0);

    private final SparkMaxSim rollerMotorSim = new SparkMaxSim(rollerMotor, rollerMotorModel);

    public IntakeRollerSubsystem() {

        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(false)
                .smartCurrentLimit(100)
                .idleMode(IdleMode.kCoast);

        rollerMotor.configure(
                config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        // Default: stopped intake
        setDefaultCommand(stop());
    }

    /*
     * =======================
     * Simulation
     * =======================
     */
    @Override
    public void simulationPeriodic() {
        // Apply motor voltage
        rollerSim.setInput(
                rollerMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Update physics (20 ms)
        rollerSim.update(0.02);

        // Push sim state back into Spark
        rollerMotorSim.iterate(
                rollerSim.getAngularVelocityRPM(),
                RoboRioSim.getVInVoltage(),
                0.02);
    }

    /*
     * =======================
     * Commands
     * =======================
     */

    /** Run intake at a duty cycle */
    public Command set(double speed) {
        return run(() -> rollerMotor.set(speed));
    }

    /** Intake inwards */
    public Command in(double speed) {
        return set(Math.abs(speed));
    }

    /** Intake outwards */
    public Command out(double speed) {
        return set(-Math.abs(speed));
    }

    /** Stop intake */
    public Command stop() {
        return set(0.0);
    }

    /*
     * =======================
     * Telemetry / State
     * =======================
     */

    public double getDutyCycle() {
        return rollerMotor.getAppliedOutput();
    }

    public Current getCurrent() {
        return Amps.of(rollerMotor.getOutputCurrent());
    }

    public boolean isRunning() {
        return Math.abs(getDutyCycle()) > 0.05;
    }

    public boolean outtaking() {
        return getDutyCycle() < -0.05;
    }
}
