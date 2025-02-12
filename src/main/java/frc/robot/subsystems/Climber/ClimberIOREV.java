package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.GEAR_RATIO;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;

public class ClimberIOREV implements ClimberIO {
  private Angle setpoint;
  protected static final double GEAR_RATIO = 12.0;

  protected final SparkFlex motor =
      new SparkFlex(ClimberConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  protected final RelativeEncoder encoder = motor.getEncoder();

  private final SparkClosedLoopController pidController;

  public ClimberIOREV() {
    pidController = motor.getClosedLoopController();

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig
        .encoder
        .velocityConversionFactor((1.0 / GEAR_RATIO) / 60.0)
        .positionConversionFactor(1.0 / GEAR_RATIO);
    motorConfig
        .closedLoop
        .outputRange(-0.5, 0.5)
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
  public void setPosition(Angle angle) {
    pidController.setReference(angle.in(Rotations), SparkBase.ControlType.kPosition);
  }
}
