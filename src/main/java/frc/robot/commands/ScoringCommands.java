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
    return Commands.sequence(arm.L1(), elevator.intake()).withName("bottomLevel");

    //    return new FunctionalCommand(
    //            () -> Commands.waitUntil(claw::hasCoral),
    //            () -> arm.L1().alongWith(elevator.intake()),
    //            onEnd -> {},
    //            () -> elevator.isAtTargetPos())
    //        .withName("bottomLevel");
  }

  public Command midLevel() {
    return Commands.sequence(arm.L2(), elevator.L3()).withName("midLevel");

    //    return new FunctionalCommand(
    //            () -> Commands.waitUntil(claw::hasCoral),
    //            () -> arm.L2().alongWith(elevator.L3()),
    //            onEnd -> {},
    //            () -> elevator.isAtTargetPos())
    //        .withName("midLevel");
  }

  public Command topLevel() {
    return Commands.sequence(arm.L2(), elevator.L4())
        // claw.hold().until(() -> !claw.hasCoral())
        .withName("topLevel");

    //    return new FunctionalCommand(
    //            () -> Commands.waitUntil(claw::hasCoral),
    //            () -> arm.L2().alongWith(elevator.L4()),
    //            onEnd -> {},
    //            () -> elevator.isAtTargetPos())
    //        .withName("topLevel");
  }

  public Command intakeCoral() {
    return Commands.sequence(
            // Still would want to wait for elevator to have game

            elevator.intake(), arm.intake(), Commands.waitUntil(elevatorHasGamePiece()))
        //            Commands.sequence(elevator.park(), claw.intake().until(claw::hasCoral))
        //                .onlyWhile(elevator::hasCoral))
        //                    claw.hold())
        .withName("intake");

    //    return new FunctionalCommand(
    //            () -> Commands.waitUntil(() -> elevator.hasCoral()),
    //            () -> claw.intake().alongWith(elevator.intake()).alongWith(arm.intake()),
    //            onEnd -> {},
    //            claw::hasCoral)
    //        .withName("intake");
  }

  public Command score() {
    return Commands.sequence(
            arm.L1(),
            Commands.waitSeconds(0.25),
            claw.extake().until(() -> !claw.hasCoral()),
            elevator.intake(), // if this causes something to get stuck, we could do
            // elevator.stop().andThen(elevator.intake()), this would unschedule the
            // last command for 20ms and then reschedule the one we want, kind of
            // resetting it so its less error prone
            arm.intake(),
            intakeCoral())
        .withName("scoreCoral");

    //    return new FunctionalCommand(
    //            () -> claw.stopClaw(),
    //            () ->
    //                arm.L1()
    //                    .andThen(Commands.waitSeconds(0.25))
    //                    .andThen(claw.extake().until(() -> !claw.hasCoral())),
    //            onEnd -> intakeCoral(),
    //            () -> !claw.hasCoral())
    //        .withName("scoreCoral");
  }

  @AutoLogOutput
  private Trigger elevatorHasGamePiece() {
    var elevatorHasGamePiece = new Trigger(elevator::hasCoral);
    // Auto moves elevator to intake when it has game piece (this logic seems weird but is what you
    // have above)
    elevatorHasGamePiece.onTrue(elevator.park().andThen(claw.intake().until(claw::hasCoral)));
    // Auto move arm and elevator as soon as elevator and claw don't have a game piece
    // elevatorHasGamePiece.and(claw::hasCoral).onFalse(arm.intake().andThen(elevator.intake()));
    return elevatorHasGamePiece;
  }

  public Command stopAll() {
    return Commands.sequence(claw.stopClaw(), arm.stopCommand(), elevator.stopCommand())
        .withName("stopAll");
  }
}
