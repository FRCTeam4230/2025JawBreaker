package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.I2C;

public class ClawIOREV implements ClawIO {
  protected final SparkFlex motor =
      new SparkFlex(ClawConstants.clawMotorID, SparkLowLevel.MotorType.kBrushless);

  protected final ColorSensorV3 colorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);

  public ClawIOREV() {
    motor.configure(
        new SparkMaxConfig()
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
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
  public void setSpeed(double angularVelocity) {

    motor.set(angularVelocity);
  }

  public boolean hasCoral() {
    return colorSensorV3.getProximity() > 1;
  }

  public void stop() {
    motor.set(0);
  }

  public void updateInputs(ClawIOInputs inputs) {
    inputs.proximity = Centimeters.of(colorSensorV3.getProximity());
    inputs.motorVelocity = RotationsPerSecond.of(motor.getEncoder().getVelocity());
    inputs.appliedVoltage = Volts.of(motor.getBusVoltage());
    inputs.supplyCurrent = Amps.of(motor.getOutputCurrent());
    inputs.motorTempCelsius = Celsius.of(motor.getMotorTemperature());
  }
}
