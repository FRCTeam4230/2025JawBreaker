package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public abstract class DefaultCurrentCommandLoggableSubsystem extends SubsystemBase {

  @Override
  public void periodic() {
    String subsytemName = getName();
    if (getCurrentCommand() != null) {
      Logger.recordOutput(subsytemName + "/Current Command", getCurrentCommand().getName());
      Logger.recordOutput(
          subsytemName + "/Current Command Requirements",
          getCurrentCommand().getRequirements().stream()
              .map(Object::toString)
              .collect(Collectors.joining(",")));
    } else {
      Logger.recordOutput(subsytemName + "/Current Command: ", "None");
    }

    if (getDefaultCommand() != null) {
      Logger.recordOutput(subsytemName + "/Default Command", getDefaultCommand().getName());
      Logger.recordOutput(
          subsytemName + "/Default Command Requirements",
          getDefaultCommand().getRequirements().stream()
              .map(Object::toString)
              .collect(Collectors.joining(",")));
    } else {
      Logger.recordOutput(subsytemName + "/Default Command", "None");
    }
  }
}
