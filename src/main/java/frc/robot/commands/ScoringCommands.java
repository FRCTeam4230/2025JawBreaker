package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.claw.*;
import frc.robot.subsystems.elevator.*;

public class ScoringCommands extends Command {

  Elevator elevator;
  Arm arm;
  Claw claw;

  public ScoringCommands(Elevator elevator, Arm arm, Claw claw) {
    this.elevator = elevator;
    this.arm = arm;
    this.claw = claw;
  }

  public Command bottomLevel() {
    return Commands.runOnce(() -> arm.L1());
  }

  public Command midLevel() {
    return Commands.sequence(arm.L2(), elevator.L3());
  }

  public Command topLevel() {
    return Commands.sequence(arm.L2(), elevator.L4());
  }

  public Command intakeCoral() {
    return Commands.sequence(arm.intake(), Commands.waitUntil(arm::isParked), elevator.intake());
  }

  public Command stopAll() {
    return Commands.sequence(claw.stopClaw(), arm.stopCommand(), elevator.stopCommand());
  }
}
