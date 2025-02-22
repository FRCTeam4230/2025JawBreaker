package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;

public class ScoringCommands extends Command {

  private final Elevator elevator;
  private final Arm arm;
  private final Claw claw;

  private final Trigger elevatorHasGamePiece;

  public ScoringCommands(Elevator elevator, Arm arm, Claw claw) {
    this.elevator = elevator;
    this.arm = arm;
    this.claw = claw;

    this.elevatorHasGamePiece = elevatorHasGamePiece();
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
        // Still would want to wait for elevator to have game
        Commands.waitUntil(elevatorHasGamePiece::getAsBoolean),
        claw.intake().until(claw::hasCoral),
        claw.hold());
  }

  private Trigger elevatorHasGamePiece() {
    var elevatorHasGamePiece = new Trigger(elevator::hasCoral);
    // Auto moves elevator to intake when it has game piece (this logic seems weird but is what you
    // have above)
    elevatorHasGamePiece.onTrue(elevator.intake());
    // Auto move arm and elevator as soon as elevator and claw don't have a game piece
    elevatorHasGamePiece
        .and(claw::hasCoral)
        .onFalse(Commands.sequence(arm.intake(), elevator.intake()));
    return elevatorHasGamePiece;
  }

  public Command stopAll() {
    return Commands.sequence(claw.stopClaw(), arm.stopCommand(), elevator.stopCommand());
  }
}
