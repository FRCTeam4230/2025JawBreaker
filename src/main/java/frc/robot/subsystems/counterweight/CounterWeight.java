package frc.robot.subsystems.counterweight;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

/** //TODO
 * CDhange this subsystem to use a pid to drive the counterwewight out to it's end point.
 */
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

  public final Command counterWeightOut() {
    return Commands.startEnd(() -> setPosition(Volts.of(4)), () -> stop());
  }

  public final Command counterWeightStop() {
    return Commands.runOnce(() -> stop());
  }
}
