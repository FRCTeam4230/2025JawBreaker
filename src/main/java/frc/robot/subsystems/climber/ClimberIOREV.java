package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOREV implements ClimberIO {
  private Angle setpoint;

  protected final SparkFlex motor =
      new SparkFlex(ClimberConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  protected final RelativeEncoder encoder = motor.getEncoder();

  private final SparkClosedLoopController pidController;

  public ClimberIOREV() {
    pidController = motor.getClosedLoopController();

    SparkFlexConfig motorConfig = new SparkFlexConfig();

    motorConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .inverted(true)
        .encoder
        .velocityConversionFactor((1.0 / GEAR_RATIO) / 60.0)
        .positionConversionFactor(1.0 / GEAR_RATIO);
    motorConfig
        .closedLoop
        .outputRange(-0.8, 0.8)
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .p(ClimberConstants.kP.get())
        .i(ClimberConstants.kI.get())
        .d(ClimberConstants.kD.get());

    motor.configure(
        motorConfig,
        SparkFlex.ResetMode.kResetSafeParameters,
        SparkFlex.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.motorConnected = true;
    inputs.encoderConnected = true;

    inputs.motorPosition = Rotations.of(encoder.getPosition());
    inputs.motorVelocity = RotationsPerSecond.of(encoder.getVelocity());

    inputs.setpoint = setpoint;

    inputs.motorStatorCurrent = Amps.of(motor.getOutputCurrent());
    inputs.motorSupplyCurrent = Amps.of(motor.getOutputCurrent());
  }

  @Override
  public void setPosition(Voltage volts) {
    //    pidController.setReference(angle.in(Rotations), SparkBase.ControlType.kPosition);
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
