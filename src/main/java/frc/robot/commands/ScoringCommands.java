package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.claw.*;



public class ScoringCommands extends Command {

    Elevator elevator;
    Arm arm;
    Claw claw;

    public Command bottomLevel() {
        return Commands.runOnce( () -> arm.L1());
    }
    public Command midLevel() {
        return Commands.sequence(arm.L2().andThen(elevator.L3()));
    }
    public Command topLevel() {
        return Commands.sequence(arm.L2().andThen(elevator.L4()));
    }
    public Command intakeCoral() {
        return Commands.sequence(arm.intake().andThen(elevator.intake().andThen(claw.intake().onlyWhile(() -> !claw.hasCoral()))));
    }
    public Command extakeCoral() {
        return Commands.sequence(intakeCoral().andThen(claw.extake().onlyWhile(() -> claw.hasCoral())));
    }

    public Command stopAll() {
        return Commands.sequence(claw.stopClaw().andThen(arm.stopCommand()).andThen(elevator.stopCommand()));
    }
}
