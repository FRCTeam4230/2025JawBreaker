package frc.robot.subsystems.counterweight;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CounterWeight extends SubsystemBase {

  private final CounterWeightIO io;
  private final CounterWeightIOInputsAutoLogged inputs;

  public CounterWeight(CounterWeightIO io) {
    this.io = io;
    this.inputs = new CounterWeightIOInputsAutoLogged();
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

  public final Command counterWeightOut(Voltage volts) {
    return Commands.startEnd(() -> setPosition(volts), () -> stop());
  }

  public final Command counterWeightStop() {
    return Commands.runOnce(() -> stop());
  }
}
