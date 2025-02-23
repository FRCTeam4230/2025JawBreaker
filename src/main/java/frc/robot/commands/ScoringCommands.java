package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.AutoLogOutput;

public class ScoringCommands {

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
    return Commands.runOnce(() -> arm.L1()).withName("bottomLevel");
  }

  public Command midLevel() {
    return Commands.sequence(arm.L2(), elevator.L3()).withName("midLevel");
  }

  public Command topLevel() {
    return Commands.sequence(
        Commands.waitUntil(claw::hasCoral)
            .andThen(arm.L2().alongWith(elevator.L4()).withName("topLevel")));
  }

  public Command intakeCoral() {
    return Commands.sequence(
            // Still would want to wait for elevator to have game
            Commands.waitUntil(elevator::hasCoral),
            claw.intake()
                .alongWith(elevator.intake())
                .alongWith(arm.intake())
                .until(claw::hasCoral),
            claw.hold())
        // .withTimeout(3)
        .withName("intake");
  }

  @AutoLogOutput
  private Trigger elevatorHasGamePiece() {
    var elevatorHasGamePiece = new Trigger(elevator::hasCoral);
    // Auto moves elevator to intake when it has game piece (this logic seems weird but is what you
    // have above)
    // elevatorHasGamePiece.onTrue(elevator.park());
    // Auto move arm and elevator as soon as elevator and claw don't have a game piece
    // elevatorHasGamePiece.and(claw::hasCoral).onFalse(arm.intake().andThen(elevator.intake()));
    return elevatorHasGamePiece;
  }

  public Command stopAll() {
    return Commands.sequence(claw.stopClaw(), arm.stopCommand(), elevator.stopCommand())
        .withName("stopAll");
  }
}
