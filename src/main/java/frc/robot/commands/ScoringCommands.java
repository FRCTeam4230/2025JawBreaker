package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;

public class ScoringCommands extends Command {

  private final Elevator elevator;
  private final Arm arm;
  private final Claw claw;

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
    return Commands.sequence(
        Commands.waitUntil(claw::doesNotHaveCoral),
        arm.intake(),
        Commands.waitUntil(arm::isParked),
        elevator.park(),
        Commands.waitUntil(elevator::hasCoral),
        elevator.intake(),
        claw.intake().until(claw::hasCoral),
        claw.hold());
  }

  public Command stopAll() {
    return Commands.sequence(claw.stopClaw(), arm.stopCommand(), elevator.stopCommand());
  }
}
