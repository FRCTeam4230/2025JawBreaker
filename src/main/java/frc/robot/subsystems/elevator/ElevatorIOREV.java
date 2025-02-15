package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class ElevatorIOREV implements ElevatorIO {
  /** The gear ratio between the motor and the elevator mechanism */
  protected static final double GEAR_RATIO = 12.0;
  /**
   * The radius of the elevator pulley/drum, used for converting between rotations and linear
   * distance
   */
  protected final Distance elevatorRadius = Inches.of((1 + 7.0 / 8.0) / 2);

  private Angle setpoint;

  /** Leader motor controller * */
  protected final SparkFlex leader =
      new SparkFlex(ElevatorConstants.LEADER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder leaderEncoder = leader.getExternalEncoder();

  //private final Encoder leaderExternalEncoder = new Encoder(5, 6);

  /*
  Encoder can take two ports, this gives the correct value in rotations for the elevator (when divided by -8192, which is how many ticks are in a rotation)
  ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder is what seems to be the correct thing to pass to the feedback sensor
  Maybe using ExternalEncoderConfig will work but it doesn't seem to have a way of applying the correct encoder to it, there is also something called SparkFlexExternalEncoder, but it doesn't seem to work yet
    */

  /** Follower * */
  protected final SparkFlex follower =
      new SparkFlex(ElevatorConstants.FOLLOWER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

//  private final DigitalInput upperLimitSwitch =
//      new DigitalInput(ElevatorConstants.UPPER_LIMIT_SWITCH_DIO_PORT);
//  private final DigitalInput lowerLimitSwitch =
//      new DigitalInput(ElevatorConstants.LOWER_LIMIT_SWITCH_DIO_PORT);

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
        .absoluteEncoder
        .velocityConversionFactor((1.0 / GEAR_RATIO) / 60.0)
        .positionConversionFactor(1.0 / GEAR_RATIO)
        .inverted(true);
        //.zeroOffset() TODO: get the offset and set it!
    config.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder)
        .p(ElevatorConstants.kP.get())
        .i(ElevatorConstants.kI.get())
        .d(ElevatorConstants.kD.get())
        .outputRange(-0.1, 0.1)
        .maxMotion
        .maxVelocity(ElevatorConstants.elevatorMaxVelocity)
        .maxAcceleration(ElevatorConstants.elevatorMaxAcceleration)
        .positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(0.01);
    config.limitSwitch
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchEnabled(true);
    
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

    //inputs.encoderPosition = Rotations.of(leader.getEncoder().getPosition());
    //inputs.encoderVelocity = RotationsPerSecond.of(leaderEncoder.getVelocity());
    inputs.encoderPosition = inputs.leaderPosition;
    inputs.encoderVelocity = inputs.leaderVelocity;
    inputs.setpoint = setpoint;

    inputs.dutyCycleEncoderPosition = Rotations.of(leaderEncoder.getPosition() / -8192.0); //TODO: what is this?

    inputs.lowerLimit = leader.getForwardLimitSwitch().isPressed();
    inputs.upperLimit = leader.getReverseLimitSwitch().isPressed();


    if ((inputs.lowerLimit && inputs.leaderVelocity.magnitude() < 0)
        || (inputs.upperLimit && inputs.leaderVelocity.magnitude() > 0)) {
      stop();
    }
  }

  @Override
  public void setDistance(Angle distance) {
    setpoint = distance.div(Math.PI);
    pidController.setReference(
        distance.in(Rotations) / Math.PI, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);

    // feedforward.calculate(leaderEncoder.getVelocity()));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
