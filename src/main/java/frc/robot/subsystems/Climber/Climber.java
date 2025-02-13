package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIO.ClimberIOInputs inputs;

  public Climber(ClimberIO io) {
    this.io = io;
    this.inputs = new ClimberIO.ClimberIOInputs();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setPosition(Voltage volts) {
    io.setPosition(volts);
  }

  public void stop() {
    io.stop();
  }

  public final Command climberOut(Voltage volts) {
    return Commands.startEnd(() -> setPosition(volts), () -> stop());
  }

  public final Command climberStop() {
    return Commands.runOnce(() -> stop());
  }
}
