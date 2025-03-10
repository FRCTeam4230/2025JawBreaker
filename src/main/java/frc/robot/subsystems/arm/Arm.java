// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Arm subsystem controls a dual-motor arm mechanism for game piece manipulation. It supports
 * multiple positions for different game actions and provides both open-loop and closed-loop control
 * options.
 */
public class Arm extends SubsystemBase {

  // Hardware interface and inputs
  private ArmIO io;
  private final ArmIOInputsAutoLogged inputs;

  // Current arm position mode
  private ArmMode currentMode = ArmMode.STOP;

  // Alerts for motor connection status
  private final Alert motorMotorAlert =
      new Alert("Arm motor motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Arm follower motor isn't connected", AlertType.kError);
  private final Alert encoderAlert = new Alert("Arm encoder isn't connected", AlertType.kError);

  /**
   * Creates a new Arm subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the arm
   */
  public Arm(ArmIO io) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    // updateControlConstants();

    // Update motor connection status alerts
    motorMotorAlert.set(!inputs.motorConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  /**
   * Runs the arm in closed-loop position mode to the specified angle.
   *
   * @param position The target angle position
   */
  private void setPosition(Angle position) {
    io.setPosition(position);
  }

  /** Stops the arm motors. */
  private void stop() {
    io.stop();
  }

  /**
   * Returns the current position of the arm.
   *
   * @return The current angular position
   */
  @AutoLogOutput
  public Angle getPosition() {
    return inputs.encoderPosition;
  }

  /** Enumeration of available arm positions with their corresponding target angles. */
  private enum ArmMode {
    STOP(Degrees.of(0)), // Stop the arm
    INTAKE(Degrees.of(-88)), // Arm tucked in
    PARKED(Degrees.of(-90)), // try to hold
    L1(Degrees.of(0)), //  Position for scoring in L1
    L2(Degrees.of(35)), //  Position for scoring in L2
    L3(Degrees.of(45)), // Position for scoring in L3
    L4(Degrees.of(50)); // Position for scoring in L4

    private final Angle targetAngle;
    private final Angle angleTolerance;

    ArmMode(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    ArmMode(Angle targetAngle) {
      this(targetAngle, Rotations.of(Degrees.of(1).in(Rotations))); // 2 degree default tolerance
    }
  }

  /**
   * Gets the current arm position mode.
   *
   * @return The current ArmMode
   */
  @AutoLogOutput
  public ArmMode getMode() {
    return currentMode;
  }

  /**
   * Sets a new arm mode and schedules the corresponding command.
   *
   * @param mode The desired ArmMode
   */
  private void setArmMode(ArmMode mode) {
    if (currentMode != mode) {
      currentCommand.cancel();
      currentMode = mode;
      currentCommand.schedule();
    }
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ArmMode.STOP,
              Commands.runOnce(this::stop).withName("Stop Arm"),
              ArmMode.INTAKE,
              createPositionCommand(ArmMode.INTAKE),
              ArmMode.L1,
              createPositionCommand(ArmMode.L1),
              ArmMode.L2,
              createPositionCommand(ArmMode.L2),
              ArmMode.L3,
              createPositionCommand(ArmMode.L3),
              ArmMode.L4,
              createPositionCommand(ArmMode.L4),
              ArmMode.PARKED,
              createPositionCommand(ArmMode.PARKED)),
          this::getMode);

  /**
   * Creates a command for a specific arm position that moves the arm and checks the target
   * position.
   *
   * @param mode The arm position to create a command for
   * @return A command that implements the arm movement
   */
  private Command createPositionCommand(ArmMode mode) {
    return Commands.runOnce(() -> setPosition(mode.targetAngle), this)
        .withName("Move to " + mode.toString());
  }

  /**
   * Checks if the arm is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ArmMode.STOP) return true;
    return getPosition().isNear(currentMode.targetAngle, currentMode.angleTolerance);
  }

  /**
   * Logs target angle for given mode.
   *
   * @return The target angle for the current mode
   */
  @AutoLogOutput
  private Angle targetAngle() {
    return currentMode.targetAngle;
  }

  @AutoLogOutput
  public boolean isParked() {
    return inputs.lowerLimit;
  }

  /**
   * Creates a command to set the arm to a specific mode.
   *
   * @param mode The desired arm mode
   * @return Command to set the mode
   */
  private Command setPositionCommand(ArmMode mode) {
    return Commands.runOnce(() -> setArmMode(mode)).withName("SetArmMode(" + mode.toString() + ")");
  }

  /** Factory methods for common position commands */

  /**
   * @return Command to move the arm to L4 position
   */
  public final Command park() {
    return setPositionCommand(ArmMode.PARKED);
  }
  /**
   * @return Command to move the arm to L1 scoring position
   */
  public final Command L1() {
    return setPositionCommand(ArmMode.L1)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
  }

  /**
   * @return Command to move the arm to L2 scoring position
   */
  public final Command L2() {
    return setPositionCommand(ArmMode.L2);
  }

  /**
   * @return Command to move the arm to L3 position
   */
  public final Command L3() {
    return setPositionCommand(ArmMode.L3);
  }

  /**
   * @return Command to move the arm to L4 position
   */
  public final Command L4() {
    return setPositionCommand(ArmMode.L4)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
  }

  /**
   * @return Command to intake the arm
   */
  public final Command intake() {
    return setPositionCommand(ArmMode.INTAKE);
  }

  public Command resetEncoder() {
    return Commands.sequence(
        Commands.run(() -> io.setVoltage(Volts.of(-0.4))).until(this::isParked),
        Commands.runOnce(() -> io.setVoltage(Volts.of(0)))
            .andThen(Commands.waitSeconds(0.4))
            .andThen(() -> io.resetEncoder()));
  }

  public Command reconfigPID() {
    return Commands.runOnce(io::reconfigurePID);
  }
  /**
   * @return Command to stop the arm
   */
  public final Command stopCommand() {
    return setPositionCommand(ArmMode.STOP);
  }

  // public final Command holdArmCommand(){ return () -> io.setVoltage(Volts.of(-0.5));}

  private SysIdRoutine armSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.2).per(Second),
              Volts.of(0.9),
              null,
              state -> Logger.recordOutput("Arm/SysIdArm_State", state.toString())),
          new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage), null, this));

  public Command runQStaticArmSysId(SysIdRoutine.Direction direction) {
    return armSysIdRoutine.quasistatic(direction);
  }

  public Command runDynamicArmSysId(SysIdRoutine.Direction direction) {
    return armSysIdRoutine.dynamic(direction);
  }
}
