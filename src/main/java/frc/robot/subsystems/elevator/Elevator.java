// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static java.util.Optional.*;

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
 * The Elevator subsystem controls a dual-motor elevator mechanism for game piece manipulation. It
 * supports multiple distances for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class Elevator extends SubsystemBase {
  // Hardware interface and inputs
  private ElevatorIO io = null;
  private final ElevatorIOInputsAutoLogged inputs;

  // Current elevator distance mode
  private ElevatorMode currentMode = ElevatorMode.STOP;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Elevator leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Elevator follower motor isn't connected", AlertType.kError);
  private final Alert encoderAlert =
      new Alert("Elevator encoder isn't connected", AlertType.kError);

  /**
   * Creates a new Elevator subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the elevator
   */
  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  /**
   * Runs the elevator in closed-loop distance mode to the specified angle.
   *
   * @param distance The target angle distance
   */
  private void setDistance(Angle distance) {
    io.setDistance(distance);
  }

  /** Stops the elevator motors. */
  private void stop() {
    io.stop();
  }

  /** Enumeration of available elevator distances with their corresponding target angles. */
  public enum ElevatorMode {
    STOP(Rotations.of(0)), // Stop the elevator
    INTAKE(Rotations.of(1.25)), // Elevator tucked in
    PARKED(Rotations.of(0)),
    L1(Rotations.of(0.2)), // Position for scoring in L1
    L2(Rotations.of(1.25)), // Position for scoring in L2
    L3(Rotations.of(4.8)), // Position for scoring in L3
    L4(Rotations.of(14)); // Position for scoring in L4

    private final Angle targetDistance;
    private final Angle distanceTolerance;

    /**
     * Creates a new elevator mode with a custom distance tolerance.
     *
     * @param targetDistance The target distance for this mode
     * @param distanceTolerance The allowed tolerance from the target distance
     */
    ElevatorMode(Angle targetDistance, Angle distanceTolerance) {
      this.targetDistance = targetDistance;
      this.distanceTolerance = distanceTolerance;
    }

    ElevatorMode(Angle targetDistance) {
      this(targetDistance, Rotations.of(0.02));
    }
  }

  /**
   * Gets the current elevator distance mode.
   *
   * @return The current ElevatorPosition
   */
  @AutoLogOutput
  public ElevatorMode getMode() {
    return currentMode;
  }

  /**
   * Sets a new elevator distance and schedules the corresponding command.
   *
   * @param mode The desired ElevatorPosition
   */
  private void setElevatorMode(ElevatorMode mode) {
    if (currentMode != mode) {
      currentCommand.cancel();
      currentMode = mode;
      currentCommand.schedule();
    }
  }

  public boolean hasCoral() {
    return inputs.beamBreakTriggered;
  }

  // Command that runs the appropriate routine based on the current distance
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ElevatorMode.STOP,
              Commands.runOnce(this::stop).withName("Stop Elevator"),
              ElevatorMode.INTAKE,
              createPositionCommand(ElevatorMode.INTAKE),
              ElevatorMode.PARKED,
              createPositionCommand(ElevatorMode.PARKED),
              ElevatorMode.L1,
              createPositionCommand(ElevatorMode.L1),
              ElevatorMode.L2,
              createPositionCommand(ElevatorMode.L2),
              ElevatorMode.L3,
              createPositionCommand(ElevatorMode.L3),
              ElevatorMode.L4,
              createPositionCommand(ElevatorMode.L4)),
          this::getMode);

  /**
   * Creates a command for a specific elevator distance that moves the elevator and checks the
   * target distance.
   *
   * @param mode The elevator distance to create a command for
   * @return A command that implements the elevator movement
   */
  private Command createPositionCommand(ElevatorMode mode) {
    return Commands.runOnce(() -> setDistance(mode.targetDistance), this)
        .withName("Move to " + mode.toString());
  }

  /**
   * Creates a command to set the elevator to a specific distance.
   *
   * @param mode The desired elevator mode
   * @return Command to set the mode
   */
  private Command setPositionCommand(ElevatorMode mode) {
    return Commands.runOnce(() -> setElevatorMode(mode))
        .withName("SetElevatorMode(" + mode.toString() + ")");
  }

  /**
   * Factory methods for elevator positions
   *
   * <p>Scoring positions: - L1: Low scoring - L2: Mid scoring - L3: High scoring - L4: Extended
   * high scoring
   *
   * <p>Other positions: - intake: Intake position - stop: Emergency stop
   */

  /**
   * @return Command to move the elevator to L1 scoring distance
   */
  public final Command L1() {
    return setPositionCommand(ElevatorMode.L1);
  }

  /**
   * @return Command to move the elevator to L2 scoring distance
   */
  public final Command L2() {
    return setPositionCommand(ElevatorMode.L2);
  }

  /**
   * @return Command to move the elevator to L3 distance
   */
  public final Command L3() {
    return setPositionCommand(ElevatorMode.L3);
  }

  /**
   * @return Command to move the elevator to L4 distance
   */
  public final Command L4() {
    return setPositionCommand(ElevatorMode.L4);
  }

  /**
   * @return Command to intake the elevator
   */
  public final Command intake() {
    return setPositionCommand(ElevatorMode.INTAKE);
  }

  /**
   * @return Command to park the elevator
   */
  public final Command park() {
    return setPositionCommand(ElevatorMode.PARKED);
  }

  /**
   * @return Command to stop the elevator
   */
  public final Command stopCommand() {
    return setPositionCommand(ElevatorMode.STOP);
  }

  private SysIdRoutine elevatorSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              state -> Logger.recordOutput("Elevator/SysIdElevator_State", state.toString())),
          new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage), null, this));

  public final Command runQStaticElevatorSysId(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.quasistatic(direction);
  }

  public final Command runDynamicElevatorSysId(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.dynamic(direction);
  }

  /**
   * Checks if the elevator is at its target height.
   *
   * @return true if at target height, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    return ofNullable(inputs.encoderPosition)
        .map(val -> val.isNear(currentMode.targetDistance, currentMode.distanceTolerance))
        .orElse(false);
  }

  @AutoLogOutput
  public boolean isAtTargetPos() {
    return ofNullable(inputs.encoderPosition)
        .map(val -> val.isNear(currentMode.targetDistance, currentMode.distanceTolerance))
        .orElse(false);
  }
}
