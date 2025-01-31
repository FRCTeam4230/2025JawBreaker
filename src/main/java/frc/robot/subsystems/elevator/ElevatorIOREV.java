package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utils.Conversions;

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
  private final RelativeEncoder leaderEncoder = leader.getEncoder();
  /** Follower * */
  private final SparkFlex follower = new SparkFlex(31, SparkLowLevel.MotorType.kBrushless);

  // Absolute position, upper and lower limit switch
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(3);
  private final DigitalInput upperLimitSwitch = new DigitalInput(1);
  private final DigitalInput lowerLimitSwitch = new DigitalInput(2);

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
    //maxMotion example https://github.com/BroncBotz3481/FRC2025/blob/main/src/main/java/frc/robot/subsystems/ElevatorSubsystem.java
    //NON maxmotion, (use profiledPidController from wpilib) https://github.com/frc868/2025-Ri3D/blob/main/src/main/java/frc/robot/subsystems/Elevator.java
    var config = new SparkFlexConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        .encoder
        .positionConversionFactor(ElevatorConstants.kRotaionToMeters) // Converts Rotations to Meters
        .velocityConversionFactor(ElevatorConstants.kRPMtoMPS) // Converts RPM to MPS
        // .encoder VELOCITY CONVERSION //TODO: fix this
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .p(1) //TODO convert these to tuneable numbers maybe??? or make the constant the tunable and reference it here
        .i(0)
        .d(0)
        .maxMotion
        .maxVelocity(ElevatorConstants.kElevatorMaxVelocity)
        .maxAcceleration(ElevatorConstants.kElevatorMaxAcceleration)
        .positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(0.01)
        .outputRange(-0.5, 0.5);

    if (isFollower) {
      config.follow(leader, true);
    }

    return config;
  }

  @Override
  public void setDistance(Distance distance) {
    pidController.setReference(
        Units.rotationsToDegrees(Conversions.metersToRotations(distance, 1, elevatorRadius)),
        SparkBase.ControlType.kPosition,ClosedLoopSlot.kSlot0,
            feedforward.calculate(leaderEncoder.getVelocity()));

  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
