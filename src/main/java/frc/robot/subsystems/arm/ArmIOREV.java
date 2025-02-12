package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.GEAR_RATIO;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmIOREV implements ArmIO {

  protected final SparkFlex leader =
      new SparkFlex(ArmConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  private final RelativeEncoder leaderEncoder = leader.getEncoder();

  protected Angle armSetPointAngle = Rotations.of(0);
  protected final Voltage ZERO_VOLTS = Volts.of(0);

  @AutoLogOutput
  private final SparkClosedLoopController controller = leader.getClosedLoopController();

  /*
   Construct the IO. let child classes override PID control
  */
  public ArmIOREV() {

    leader.configure(
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
        .outputRange(-0.2, 0.2);

    return motorConfig;
  }

  protected EncoderConfig getEncoderConfig() {
    return new EncoderConfig()
        .velocityConversionFactor((1.0 / GEAR_RATIO) / 60.0)
        .positionConversionFactor(1.0 / GEAR_RATIO);
  }

  public void updateInputs(ArmIOInputs inputs) {

    // In hardware we assume the devices are connected.
    inputs.leaderConnected = true;
    // inputs.followerConnected = true;
    inputs.encoderConnected = true;

    // Get the arm’s current position (in rotations) and angular velocity (rotations per second)
    // from the integrated encoder.
    double armRot = leaderEncoder.getPosition();
    double armVelRotPerSec = leaderEncoder.getVelocity();

    inputs.leaderPosition = Rotations.of(armRot);
    inputs.leaderRotorPosition = Rotations.of(armRot * GEAR_RATIO);
    inputs.leaderVelocity = RotationsPerSecond.of(armVelRotPerSec);
    inputs.leaderRotorVelocity = RotationsPerSecond.of(armVelRotPerSec * GEAR_RATIO);

    inputs.appliedVoltage =
        Volts.of(leader.getAppliedOutput() * RobotController.getBatteryVoltage());
    inputs.leaderStatorCurrent = Amps.of(leader.getOutputCurrent());
    inputs.leaderSupplyCurrent = Amps.of(leader.getOutputCurrent());

    // inputs.motorTemperatureCelsius = Celsius.of(motor.getMotorTemperature());
    // inputs.motorSensorFault = motor.getFaults().sensor;
    // inputs.motorBrownOut = motor.getFaults().other;
    inputs.motorCANID = leader.getDeviceId();
    inputs.armAngle = Rotations.of(armRot);

    inputs.motorPositionFactor = leader.configAccessor.encoder.getPositionConversionFactor();
    inputs.armSetPointAngle = this.armSetPointAngle;
  }

  @Override
  public void setPosition(Angle angle) {
    this.armSetPointAngle = angle;
    controller.setReference(angle.in(Rotations), SparkBase.ControlType.kPosition);
  }

  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void reconfigurePID() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig
        .closedLoop
        .p(ArmConstants.kP.get())
        .i(ArmConstants.kI.get())
        .d(ArmConstants.kD.get());
    leader.configure(
        motorConfig,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void resetEncoder() {
    leader.getEncoder().setPosition(0);
  }
}
