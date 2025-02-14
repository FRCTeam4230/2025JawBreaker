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

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
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
  private final Alert motorAlert = new Alert("Claw motor isn't connected", AlertType.kError);
  private final Alert encoderAlert = new Alert("Claw encoder isn't connected", AlertType.kError);
  private ClawMode currentMode = ClawMode.OFF;

  public Claw(ClawIO io) {
    this.io = io;
    this.inputs = new ClawIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);

    // Update motor connection status alerts
    motorAlert.set(!inputs.leaderConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  private void setVolts(Voltage volts) {
    io.setVolts(volts);
  }

  private void stop() {
    io.stop();
  }

  private void setClawMode(ClawMode mode) {
    if (currentMode != mode) {
      currentCommand.cancel();
      currentMode = mode;
      currentCommand.schedule();
    }
  }

  private enum ClawMode {
    INTAKE(Volts.of(ClawConstants.INTAKE_VOLTAGE.get()).times(-1)),
    EXTAKE(Volts.of(ClawConstants.INTAKE_VOLTAGE.get())),
    OFF(Volts.of(0)),
    HOLD(Volts.of(ClawConstants.HOLD_VOLTAGE.get()));

    private final Voltage volts;

    ClawMode(Voltage volts) {
      this.volts = volts;
    }
  }

  public ClawMode getMode() {
    return currentMode;
  }

  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ClawMode.OFF,
              Commands.runOnce(this::stop),
              ClawMode.INTAKE,
              setVoltsCommand(Claw.ClawMode.INTAKE),
              ClawMode.EXTAKE,
              createVoltsCommand(ClawMode.EXTAKE),
              ClawMode.HOLD,
              createVoltsCommand(Claw.ClawMode.HOLD)),
          this::getMode);

  private Command createVoltsCommand(ClawMode mode) {
    return Commands.runOnce(() -> setVolts(mode.volts));
  }

  //  public Command intake() {
  //    return Commands.startEnd(
  //        () -> io.setVolts(Volts.of(ClawConstants.INTAKE_VOLTAGE.get()).times(-1)), () ->
  // io.stop());
  //  }
  //
  //  public Command extake() {
  //    return Commands.startEnd(
  //        () -> io.setVolts(Volts.of(ClawConstants.INTAKE_VOLTAGE.get())), () -> io.stop());
  //  }

  private Command setVoltsCommand(ClawMode mode) {
    return Commands.runOnce(() -> setClawMode(mode));
  }

  public Command off() {
    return createVoltsCommand(ClawMode.OFF);
  }

  public Command intake() {
    return createVoltsCommand(ClawMode.INTAKE).until(io::hasCoral).andThen(hold());
  }

  public Command extake() {
    return createVoltsCommand(ClawMode.EXTAKE);
  }

  public Command hold() {
    return createVoltsCommand(ClawMode.HOLD);
  }
}
