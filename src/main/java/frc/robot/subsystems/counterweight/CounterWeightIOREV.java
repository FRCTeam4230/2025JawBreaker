package frc.robot.subsystems.counterweight;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class CounterWeightIOREV implements CounterWeightIO {
  private Angle setpoint;
  protected static final double GEAR_RATIO = 125.0;

  protected final SparkFlex motor =
      new SparkFlex(CounterWeightConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  protected final RelativeEncoder encoder = motor.getEncoder();

  private final SparkClosedLoopController pidController;

  public CounterWeightIOREV() {
    pidController = motor.getClosedLoopController();

    SparkFlexConfig motorConfig = new SparkFlexConfig();

    motorConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .encoder
        .velocityConversionFactor((1.0 / GEAR_RATIO) / 60.0)
        .positionConversionFactor(1.0 / GEAR_RATIO);
    motorConfig
        .closedLoop
        .outputRange(-0.5, 0.5)
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .p(CounterWeightConstants.kP.get())
        .i(CounterWeightConstants.kI.get())
        .d(CounterWeightConstants.kD.get());

    motor.configure(
        motorConfig,
        SparkFlex.ResetMode.kResetSafeParameters,
        SparkFlex.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(CounterWeightIOInputs inputs) {
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
