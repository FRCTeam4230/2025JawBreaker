package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmIOREV implements ArmIO {
  private final SparkFlex motor =
      new SparkFlex(ArmConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  PIDController pidController = new PIDController(0.0, 0.0, 0.0);

  private final RelativeEncoder velocityEncoder = motor.getEncoder();
  private final DutyCycleEncoder encoder =
      new DutyCycleEncoder(ArmConstants.DUTY_CYCLE_ENCODER_PORT);

  public ArmIOREV() {

    EncoderConfig encoderConfig =
        new EncoderConfig()
            .velocityConversionFactor(Math.PI * 2 / 60 / ArmConstants.MOTOR_TO_ARM_RATIO);

    EncoderConfig relativeEncoderConfig = new EncoderConfig().positionConversionFactor(2 * Math.PI);
    motor.configure(
        new SparkFlexConfig().idleMode(SparkBaseConfig.IdleMode.kBrake).apply(encoderConfig),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.motorPosition =
        Radian.of(
            (encoder.get() * Math.PI * 2)
                - ArmConstants.ARM_ENCODER_OFFSET_RAD); // offset in radians but could need degree
    inputs.motorVelocity =
        RevolutionsPerSecond.of(
            velocityEncoder.getVelocity()); // currently rps, could potentially be rpm
    inputs.appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.motorSupplyCurrent = Amps.of(motor.getOutputCurrent());
    inputs.motorTemperatureCelsius = Celsius.of(motor.getMotorTemperature());
    // inputs.motorSensorFault = motor.getFaults().sensor; ??
    // inputs.motorBrownOut = motor.getFaults().other;
    inputs.motorCANID = motor.getDeviceId();
  }
}
