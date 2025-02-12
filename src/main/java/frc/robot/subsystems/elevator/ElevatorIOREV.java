package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOREV implements ElevatorIO {
  /** The gear ratio between the motor and the elevator mechanism */
  protected static final double GEAR_RATIO = 12.0;
  /**
   * The radius of the elevator pulley/drum, used for converting between rotations and linear
   * distance
   */
  protected final Distance elevatorRadius = Inches.of(1);

  /** Leader motor controller * */
  protected final SparkFlex leader =
      new SparkFlex(ElevatorConstants.LEADER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  private final RelativeEncoder leaderEncoder = leader.getEncoder();

  /** Follower * */
  protected final SparkFlex follower =
      new SparkFlex(ElevatorConstants.FOLLOWER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  private final DigitalInput upperLimitSwitch =
      new DigitalInput(ElevatorConstants.UPPER_LIMIT_SWITCH_DIO_PORT);
  private final DigitalInput lowerLimitSwitch =
      new DigitalInput(ElevatorConstants.LOWER_LIMIT_SWITCH_DIO_PORT);

  private final SparkClosedLoopController pidController;
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

  public ElevatorIOREV() {
    leader.configure(
        createSparkFlexConfig(false),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    follower.configure(
        createSparkFlexConfig(true),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    pidController = leader.getClosedLoopController();
  }

  private SparkFlexConfig createSparkFlexConfig(boolean isFollower) {

    // I don't know if we will need a velocityConversionFacdtory but i'm fairly certain we will
    // maxMotion example
    // https://github.com/BroncBotz3481/FRC2025/blob/main/src/main/java/frc/robot/subsystems/ElevatorSubsystem.java
    // NON maxmotion, (use profiledPidController from wpilib)
    // https://github.com/frc868/2025-Ri3D/blob/main/src/main/java/frc/robot/subsystems/Elevator.java
    var config = new SparkFlexConfig();

    config.idleMode(SparkBaseConfig.IdleMode.kCoast);
    config.inverted(true);
    config
        .encoder
        .velocityConversionFactor((1.0 / GEAR_RATIO) / 60.0)
        .positionConversionFactor(1.0 / GEAR_RATIO);
    config
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .p(ElevatorConstants.kP.get())
        .i(ElevatorConstants.kI.get())
        .d(ElevatorConstants.kD.get())
        .outputRange(-0.5, 0.5)
        .maxMotion
        .maxVelocity(ElevatorConstants.elevatorMaxVelocity)
        .maxAcceleration(ElevatorConstants.elevatorMaxAcceleration)
        .positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(0.01);

    if (isFollower) {
      config.follow(leader, true);
    }

    return config;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    double elevatorRot = leaderEncoder.getPosition();
    // this is the encoder position
    // rev does it have something like this? inputs.leaderRotorPosition = leader.get //
    double elevatorVelRotPerSec = leaderEncoder.getVelocity();
    // getVelocity returns RPMS so this is totally probably wrong

    inputs.leaderPosition = Rotations.of(elevatorRot);
    inputs.leaderVelocity = RotationsPerSecond.of(elevatorVelRotPerSec);

    inputs.appliedVoltage = Volts.of(leader.getAppliedOutput());
    inputs.leaderStatorCurrent = Amps.of(leader.getOutputCurrent());

    inputs.followerStatorCurrent = Amps.of(follower.getOutputCurrent());
    inputs.encoderPosition = Rotations.of(leader.getEncoder().getPosition());
    inputs.encoderVelocity = RotationsPerSecond.of(leaderEncoder.getVelocity());

    inputs.lowerLimit = !lowerLimitSwitch.get();
    inputs.upperLimit = !upperLimitSwitch.get();

    if (inputs.upperLimit) {
      stop();
    }
    if (inputs.lowerLimit && inputs.leaderVelocity.magnitude() < 0) {
      stop();
    }
    //    else if (inputs.lowerLimit) {
    //      leaderEncoder.setPosition(0);
    //    }
  }

  @Override
  public void setDistance(Distance distance) {
    double rotations = distance.in(Meters) / elevatorRadius.in(Meters) / GEAR_RATIO;
    pidController.setReference(rotations, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    // feedforward.calculate(leaderEncoder.getVelocity()));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
