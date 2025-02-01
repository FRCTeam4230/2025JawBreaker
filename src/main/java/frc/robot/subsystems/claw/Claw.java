// Copyright FRC 5712
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.claw;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Arm subsystem controls a dual-motor arm mechanism for game piece manipulation. It supports
 * multiple positions for different game actions and provides both open-loop and closed-loop control
 * options.
 */
public class Claw extends SubsystemBase {
  // Hardware interface and inputs
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs;


  // Alerts for motor connection status
  private final Alert motorAlert =
      new Alert("Claw motor isn't connected", AlertType.kError);
  private final Alert encoderAlert = new Alert("Arm encoder isn't connected", AlertType.kError);

  /**
   * Creates a new Arm subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the arm
   */
  public Claw(ClawIO io) {
    this.io = io;
    this.inputs = new ClawIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Update motor connection status alerts
    motorAlert.set(!inputs.leaderConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  /**
   * Runs the arm in closed-loop position mode to the specified velocity
   *
   * @param velocity The target angle position
   */
  private void setVelocity(Velocity velocity) {
    io.setVelocity(velocity);
  }

  /** Stops the arm motors. */
  private void stop() {
    io.stop();
  }


  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ArmPosition.STOP,
              Commands.runOnce(this::stop).withName("Stop Arm"),
              ArmPosition.INTAKE,
              createPositionCommand(ArmPosition.INTAKE),
              ArmPosition.L1,
              createPositionCommand(ArmPosition.L1),
              ArmPosition.L2,
              createPositionCommand(ArmPosition.L2),
              ArmPosition.L3,
              createPositionCommand(ArmPosition.L3),
              ArmPosition.L4,
              createPositionCommand(ArmPosition.L4)),
          this::getMode);

  /**
   * Creates a command for a specific arm position that moves the arm and checks the target
   * position.
   *
   * @param position The arm position to create a command for
   * @return A command that implements the arm movement
   */
  private Command createPositionCommand(ArmPosition position) {
    return Commands.runOnce(() -> setPosition(position.targetAngle))
        .withName("Move to " + position.toString());
  }

  /**
   * Checks if the arm is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ArmPosition.STOP) return true;
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

  /**
   * Creates a command to set the arm to a specific position.
   *
   * @param position The desired arm position
   * @return Command to set the position
   */
  private Command setVelocityCommand(Direction position) {
    return Commands.runOnce(() -> setArmPosition(position))
        .withName("SetArmPosition(" + position.toString() + ")");
  }

  /** Factory methods for common velocity commands */

  /**
   * @return Command to move the arm to L4 position
   */
  public final Command extake() {
    return setPositionCommand(ArmPosition.L4);
  }

  /**
   * @return Command to intake the arm
   */
  public final Command intake() {
    return setPositionCommand(ArmPosition.INTAKE);
  }

  /**
   * @return Command to stop the arm
   */
  public final Command stopCommand() {
    return setPositionCommand(ArmPosition.STOP);
  }
}
