package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmIOREV implements ArmIO {

  protected static final SparkFlex motor =
      new SparkFlex(ArmConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private SparkFlexConfig motorConfig;

  private final RelativeEncoder velocityEncoder = motor.getEncoder();

  protected final DutyCycleEncoder encoder =
      new DutyCycleEncoder(ArmConstants.DUTY_CYCLE_ENCODER_PORT);

  private SparkClosedLoopController controller;

  public ArmIOREV() {

    controller = motor.getClosedLoopController();

    EncoderConfig encoderConfig =
        new EncoderConfig()
            .velocityConversionFactor(Math.PI * 2 / 60 / ArmConstants.GEAR_RATIO)
            .positionConversionFactor(Math.PI * 2 / ArmConstants.GEAR_RATIO);

    EncoderConfig relativeEncoderConfig = new EncoderConfig().positionConversionFactor(2 * Math.PI);
    motor.configure(
        new SparkFlexConfig().idleMode(SparkBaseConfig.IdleMode.kBrake).apply(encoderConfig),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    motorConfig = new SparkFlexConfig();
    motorConfig
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.1, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motor.configure(
        motorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    motor.getEncoder().setPosition(0);
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.motorPosition =
        Rotations.of(
            motor.getEncoder().getPosition()
                - ArmConstants.ARM_ENCODER_OFFSET_RAD); // offset in radians but could need degree
    inputs.motorVelocity = RotationsPerSecond.of(velocityEncoder.getVelocity()).div(60);

    inputs.appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.motorSupplyCurrent = Amps.of(motor.getOutputCurrent());

    inputs.motorTemperatureCelsius = Celsius.of(motor.getMotorTemperature());
    // inputs.motorSensorFault = motor.getFaults().sensor;
    // inputs.motorBrownOut = motor.getFaults().other;
    inputs.motorCANID = motor.getDeviceId();

    inputs.encoderPosition = Rotations.of(encoder.get());
    inputs.encoderPositionInRadians = Radians.of(inputs.encoderPosition.in(Radians));
    inputs.encoderVelocity = RotationsPerSecond.of(velocityEncoder.getVelocity()).div(60);

    inputs.armAngle = inputs.encoderPosition;

  }

  @Override
  public void setPosition(Angle angle) {
    controller.setReference(-(angle.in(Degrees)), SparkBase.ControlType.kPosition); // inverted
  }

  public void stop() {
    motor.stopMotor();
  }
}
