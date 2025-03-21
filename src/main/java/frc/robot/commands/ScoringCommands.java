package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.lights.LightsCTRE;

public class ScoringCommands {

  private final Elevator elevator;
  private final Arm arm;
  private final Claw claw;
  private final LightsCTRE lights;

  //  private final Trigger elevatorHasGamePiece;

  public ScoringCommands(Elevator elevator, Arm arm, Claw claw) {
    this.elevator = elevator;
    this.arm = arm;
    this.claw = claw;
    this.lights = new LightsCTRE();

    //    this.elevatorHasGamePiece = elevatorHasGamePiece();
  }

  public Command bottomLevel() {
    return Commands.sequence(arm.park(), elevator.park()).withName("bottomLevel");
    // .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);

    //    return new FunctionalCommand(
    //            () -> Commands.waitUntil(claw::hasCoral),
    //            () -> arm.L1().alongWith(elevator.intake()),
    //            onEnd -> {},
    //            () -> elevator.isAtTargetPos())
    //        .withName("bottomLevel");
  }

  public Command midLevel() {
    return Commands.sequence(arm.L2(), elevator.L3()).withName("midLevel");
    // .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);

    //    return new FunctionalCommand(
    //            () -> Commands.waitUntil(claw::hasCoral),
    //            () -> arm.L2().alongWith(elevator.L3()),
    //            onEnd -> {},
    //            () -> elevator.isAtTargetPos())
    //        .withName("midLevel");
  }

  public Command topLevel() {
    return Commands.sequence(arm.L2(), elevator.L4());
    // .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
    // claw.hold().until(() -> !claw.hasCoral())
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

        elevator.intake(),
        arm.intake(),
        Commands.waitUntil(elevator::hasCoral),
        elevator.park(),
        claw.intake().until(claw::hasCoral),
        //        Commands.runOnce(() ->
        // lights.changeAnimation(LightsCTRE.AnimationTypes.SingleFadeGreen)),
        //                Commands.runOnce(() -> lights.flashColour("green"), lights),
        //                Commands.runOnce(() -> lights.setZero(), lights),
        arm.L3());
  }

  public Command score() {
    return Commands.sequence(
        arm.L1(),
        Commands.waitSeconds(0.4),
        claw.extake().until(() -> !claw.hasCoral()),
        //        Commands.runOnce(() ->
        // lights.changeAnimation(LightsCTRE.AnimationTypes.SingleFadeRed)),
        elevator.intake(),
        arm.intake(),
        intakeCoral());
    // .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
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

  //  private Trigger elevatorHasGamePiece() {
  //    var elevatorHasGamePiece = new Trigger(elevator::hasCoral);
  //    // Auto moves elevator to intake when it has game piece (this logic seems weird but is what
  // you
  //    // have above)
  //    elevatorHasGamePiece.onTrue(elevator.park().andThen(claw.intake().until(claw::hasCoral)));
  //    // Auto move arm and elevator as soon as elevator and claw don't have a game piece
  //    //
  // elevatorHasGamePiece.and(claw::hasCoral).onFalse(arm.intake().andThen(elevator.intake()));
  //    return elevatorHasGamePiece;
  //  }

  public Command stopAll() {
    return Commands.sequence(claw.stopClaw(), arm.stopCommand(), elevator.stopCommand())
        .withName("stopAll");
  }
}
