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
import frc.robot.subsystems.DefaultCurrentCommandLoggableSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * The Arm subsystem controls a dual-motor arm mechanism for game piece manipulation. It supports
 * multiple positions for different game actions and provides both open-loop and closed-loop control
 * options.
 */
public class Claw extends DefaultCurrentCommandLoggableSubsystem {
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
    super.periodic(); // LOG commands

    // Update and log inputs from hardware
    io.updateInputs(inputs);

    Logger.processInputs("Claw", inputs);
    // Update motor connection status alerts
    motorAlert.set(!inputs.leaderConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  public Command hold() {
    return Commands.run(() -> io.setVolts(Volts.of(0.5)), this);
  }

  public Command intake() {
    return Commands.run(() -> io.setVolts(Volts.of(ClawConstants.INTAKE_VOLTAGE.get())), this);
  }
  // TODO: smart current limit for motor, set hold mode to only enable when proximity gets too far

  public Command extake() {
    return Commands.run(
        () ->
            Commands.startEnd(
                () -> io.setVolts(Volts.of(ClawConstants.INTAKE_VOLTAGE.get()).times(-0.5)),
                () -> io.stop()),
        this);
  }

  public Command stopClaw() {
    return Commands.runOnce(() -> io.stop());
  }

  public boolean hasCoral() {
    return inputs.beamBreakTriggered;
  }

  public boolean doesNotHaveCoral() {
    return !inputs.beamBreakTriggered;
  }
}
