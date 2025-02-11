// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.arm.ArmConstants.GEAR_RATIO;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

/**
 * REV-based implementation of the ArmIO interface. This class uses two SparkFlex motor controllers
 * (a leader and a follower) that control a SparkFlex brushless motor. The leader’s integrated
 * encoder is configured to report arm position (in rotations) by applying an appropriate conversion
 * factor.
 *
 * <p>Note: PID (and any feedforward) gains must be tuned for your mechanism.
 */
public class ArmIOREV implements ArmIO {
  /** The gear ratio between motor and arm (for converting motor rotations to arm angle) */

  /** Leader motor controller (CAN ID 20) */
  public final SparkFlex leader = new SparkFlex(ArmConstants.MOTOR_ID, MotorType.kBrushless);

  /**
   * The SparkMax’s built–in relative encoder is used to determine the leader’s position. (An
   * absolute encoder could be used if available.)
   */
  public final RelativeEncoder leaderEncoder = leader.getEncoder();

  private SparkClosedLoopController closedLoopController = leader.getClosedLoopController();

  /**
   * Constructs a new ArmIOREV instance.
   *
   * <p>This constructor configures the leader and follower (sets follower to follow the leader),
   * sets the motor controllers’ idle mode, configures the integrated encoder conversion factors,
   * and applies example PID gains.
   */
  public ArmIOREV() {
    // Set both motors to coast mode
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    leaderConfig
        .inverted(true)
        .encoder
        .velocityConversionFactor((1.0 / GEAR_RATIO) / 60.0) // Converts RPM to rotations per second
        .positionConversionFactor(1.0 / GEAR_RATIO); // Converts motor rotations to arm rotations
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ArmConstants.kP.get())
        .i(ArmConstants.kI.get())
        .d(ArmConstants.kP.get());

    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Updates the ArmIOInputs with the latest sensor readings from the SparkMax.
   *
   * @param inputs The ArmIOInputs object to update.
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // In hardware we assume the devices are connected.
    inputs.motorConnected = true;
    // inputs.followerConnected = true;
    inputs.encoderConnected = true;

    // Get the arm’s current position (in rotations) and angular velocity (rotations per second)
    // from the integrated encoder.
    double armRot = leaderEncoder.getPosition();
    double armVelRotPerSec = leaderEncoder.getVelocity();

    inputs.motorPosition = Rotations.of(armRot);
    // For the motor’s rotor position we reconstruct the raw (pre–conversion) value.
    inputs.motorVelocity = RotationsPerSecond.of(armVelRotPerSec);

    // The applied voltage is the output percentage multiplied by the current battery voltage.
    inputs.appliedVoltage =
        Volts.of(leader.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Currents are read from the SparkMax.
    inputs.motorStatorCurrent = Amps.of(leader.getOutputCurrent());
    // inputs.followerStatorCurrent = Amps.of(follower.getOutputCurrent());
    // For demonstration, assume the supply current is similar.
    inputs.motorSupplyCurrent = Amps.of(leader.getOutputCurrent());
    // inputs.followerSupplyCurrent = Amps.of(follower.getOutputCurrent());

    // Use the integrated encoder measurement as the arm’s current angle.
    inputs.armAngle = Rotations.of(armRot);

  }

  /**
   * Commands the arm to move to the specified angle.
   *
   * <p>This implementation uses the leader SparkMax’s built–in PID controller.
   *
   * @param angle The target arm angle (in rotations) as an Angle object.
   */
  @Override
  public void setPosition(Angle angle) {
    // The setpoint is in rotations.
    closedLoopController.setReference(angle.in(Rotations), ControlType.kPosition);
  }

  /** Stops all arm movement. */
  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void reconfigurePID() {
    // Reconfigure the PID gains for the closed-loop controller.
    // Note: This is a demonstration and should be tuned for your specific mechanism.
    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    leaderConfig
        .closedLoop
        .p(ArmConstants.kP.get())
        .i(ArmConstants.kI.get())
        .d(ArmConstants.kD.get());
    leader.configure(
        leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}
