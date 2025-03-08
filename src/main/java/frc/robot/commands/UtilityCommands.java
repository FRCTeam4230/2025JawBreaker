package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class UtilityCommands {
  public static Command flashLimelights(){
    return Commands.runOnce( () -> {
      Constants.limelightCameras.forEach(camera ->
          LimelightHelpers.setLEDMode_ForceBlink(camera));
        }).andThen(Commands.runOnce(() -> {
          Constants.limelightCameras.forEach(camera ->
          LimelightHelpers.setLEDMode_ForceOff(camera));
        }));
  }
}

