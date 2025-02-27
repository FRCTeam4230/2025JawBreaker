package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClawIOREV implements ClawIO {
  protected final SparkMax motor =
      new SparkMax(ClawConstants.clawMotorID, SparkLowLevel.MotorType.kBrushless);

  protected final DigitalInput beamBreakSensor = new DigitalInput(ClawConstants.beamBreakDIOPort);

  public ClawIOREV() {
    motor.configure(
        new SparkMaxConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .voltageCompensation(12.0)
            .smartCurrentLimit(30)
            .closedLoopRampRate(ClawConstants.CLOSED_LOOP_RAMP_RATE)
            .openLoopRampRate(ClawConstants.OPEN_LOOP_RAMP_RATE),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  /*
  public void rollerSpeed(double speed) {
    motor.set(speed);
  }
  */
  @Override
  public void setVolts(Voltage voltage) {

    // motor.set(angularVelocity);
    motor.setVoltage(voltage);
  }

  public void hold() {}

  public void stop() {
    motor.set(0);
  }

  public void updateInputs(ClawIOInputs inputs) {
    inputs.beamBreakTriggered = !beamBreakSensor.get();

    inputs.leaderConnected = true;
    inputs.encoderConnected = true;

    inputs.motorVelocity = RotationsPerSecond.of(motor.getEncoder().getVelocity());
    inputs.appliedVoltage = Volts.of(motor.getAppliedOutput());
    inputs.supplyCurrent = Amps.of(motor.getOutputCurrent());
    inputs.motorTempCelsius = Celsius.of(motor.getMotorTemperature());
  }
}
