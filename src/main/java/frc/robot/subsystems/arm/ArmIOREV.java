package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmIOREV implements ArmIO {

  protected static final SparkFlex motor =
      new SparkFlex(ArmConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder velocityEncoder = motor.getEncoder();
  protected final DutyCycleEncoder encoder =
      new DutyCycleEncoder(ArmConstants.DUTY_CYCLE_ENCODER_PORT);

  @AutoLogOutput
  private final SparkClosedLoopController controller;

  /*
    Construct the IO. let child classes override PID control
   */
  public ArmIOREV() {
    controller = motor.getClosedLoopController();

    motor.configure(
        new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(60)
            .apply(getEncoderConfig())
            .apply(getMotorConfig()),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    resetEncoder();
  }

  protected SparkFlexConfig getMotorConfig() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig
        .closedLoop
        // .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(ArmConstants.kP.get())
        .i(ArmConstants.kI.get())
        .d(ArmConstants.kD.get())
        .outputRange(-0.2, 0.2)
        // Set PID values for velocity control in slot 1
        .p(0.1, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-0.2, 0.2, ClosedLoopSlot.kSlot1);

    return motorConfig;
  }

  protected EncoderConfig getEncoderConfig(){
    EncoderConfig encoderConfig =
        new EncoderConfig()
            .velocityConversionFactor((Math.PI * 2) * ArmConstants.GEAR_RATIO)
            .positionConversionFactor((Math.PI * 2) * ArmConstants.GEAR_RATIO);
    return encoderConfig;
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.motorPosition = Rotations.of(motor.getEncoder().getPosition());
    inputs.motorVelocity = RotationsPerSecond.of(velocityEncoder.getVelocity()).div(60);

    inputs.appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.motorSupplyCurrent = Amps.of(motor.getOutputCurrent());

    inputs.motorTemperatureCelsius = Celsius.of(motor.getMotorTemperature());
    // inputs.motorSensorFault = motor.getFaults().sensor;
    // inputs.motorBrownOut = motor.getFaults().other;
    inputs.motorCANID = motor.getDeviceId();

    inputs.armAngle = Rotations.of(encoder.get()).minus(ArmConstants.ARM_ENCODER_OFFSET_RAD);
    inputs.encoderPosition = Rotations.of(encoder.get());
    inputs.encoderVelocity = RotationsPerSecond.of(velocityEncoder.getVelocity()).div(60);

    //    inputs.armAngle = inputs.encoderPosition;
    inputs.motorPositionFactor = motor.configAccessor.encoder.getPositionConversionFactor();

  }

  @Override
  public void setPosition(Angle angle) {
    controller.setReference(angle.baseUnitMagnitude(), SparkBase.ControlType.kPosition);
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void reconfigurePID() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig
        .closedLoop
        .p(ArmConstants.kP.get())
        .i(ArmConstants.kI.get())
        .d(ArmConstants.kD.get());
    motor.configure(
        motorConfig,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void resetEncoder() {
    motor.getEncoder().setPosition(0);
  }
}


