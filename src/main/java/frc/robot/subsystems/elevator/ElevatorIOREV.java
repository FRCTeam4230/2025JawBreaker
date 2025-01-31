package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ElevatorIOREV implements ElevatorIO {
  /** The gear ratio between the motor and the elevator mechanism */
  private static final double GEAR_RATIO = 2.0;
  /**
   * The radius of the elevator pulley/drum, used for converting between rotations and linear
   * distance
   */
  private final Distance elevatorRadius = Inches.of(2);

  /** Leader motor controller * */
  private final SparkFlex leader = new SparkFlex(30, SparkLowLevel.MotorType.kBrushless);

  /** Follower * */
  private final SparkFlex follower = new SparkFlex(31, SparkLowLevel.MotorType.kBrushless);

  // Absolute position, upper and lower limit switch
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(3);
  private final DigitalInput upperLimitSwitch = new DigitalInput(1);
  private final DigitalInput lowerLimitSwitch = new DigitalInput(2);

  public ElevatorIOREV() {
    leader.configure(
        createSparkFlexConfig(false),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    follower.configure(
        createSparkFlexConfig(true),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public void setDistance(Angle distance) {
    //
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.leaderPosition =
        Rotations.of(leader.getEncoder().getPosition()); // this is the encoder position
    // rev does it have something like this? inputs.leaderRotorPosition = leader.get //
    inputs.leaderVelocity =
        RotationsPerSecond.of(
            leader
                .getEncoder()
                .getVelocity()); // getVelocity returns RPMS so this is totally probably wrong
    // inputs.leaderRotorVelocity = leader.getRotorVelocity();

    inputs.appliedVoltage = Volts.of(leader.getAppliedOutput());
    inputs.leaderStatorCurrent = Amps.of(leader.getOutputCurrent());

    inputs.followerStatorCurrent = Amps.of(follower.getOutputCurrent());
    inputs.encoderPosition = Rotations.of(encoder.get());
    inputs.encoderVelocity =
        RotationsPerSecond.of(
            encoder.get()); // TODO: This is probably wrong and or may not be helpful?
  }

  private SparkFlexConfig createSparkFlexConfig(boolean isFollower) {

    // I don't know if we will need a velocityConversionFacdtory but i'm fairly certain we will

    /* EncoderConfig leaderEncoderConfig =
        new EncoderConfig()
            .velocityConversionFactor(Math.PI * 2 / 60 / ArmConstants.MOTOR_TO_ARM_RATIO);
    EncoderConfig relativeEncoderConfig = new EncoderConfig().positionConversionFactor(2 * Math.PI);
    leader.configure(
        new SparkFlexConfig()
            .inverted(true)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .apply(leaderEncoderConfig),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    // The motors are mirrored, so invert
    */

    var config = new SparkFlexConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        // .encoder VELOCITY CONVERSION //TODO: fix this
        .closedLoop
        .p(24)
        .i(0)
        .d(1.6)
        .velocityFF(0);

    if (isFollower) {
      config.follow(leader, true);
    }

    // TODO: research SmartMotion and MaxMotion for gravity control
    // TODO: Look at the ARM (maybe??) from 2024Lifesaver
    /*    config.Slot0.kA = 0; // Acceleration feedforward
    config.Slot0.kG = 0.7297; // Gravity feedforward
     */

    return config;
  }
}
