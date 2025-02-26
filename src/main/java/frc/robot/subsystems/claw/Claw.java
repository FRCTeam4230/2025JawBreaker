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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  //  private ClawMode currentMode = ClawMode.OFF;

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

  public Command hold() {
    return Commands.run(() -> shouldHold(), this);
  }

  private void shouldHold() {
    if (hasCoral()) {
      io.setVolts(Volts.of(ClawConstants.HOLD_VOLTAGE.get()));
    } else {
      io.setVolts(Volts.of(0));
    }
  }

  public Command intake() {
    return Commands.startEnd(
        () -> io.setVolts(Volts.of(ClawConstants.INTAKE_VOLTAGE.get())), () -> io.stop(), this);
  }
  // TODO: smart current limit for motor, set hold mode to only enable when proximity gets too far

  public Command extake() {
    return Commands.startEnd(
        () -> io.setVolts(Volts.of(ClawConstants.INTAKE_VOLTAGE.get()).times(-0.5)),
        () -> io.stop(),
        this);
  }

  public Command stopClaw() {
    return Commands.runOnce(() -> io.stop(), this);
  }

  public boolean hasCoral() {
    return inputs.beamBreakTriggered;
  }

  public boolean doesNotHaveCoral() {
    return !inputs.beamBreakTriggered;
  }
}
