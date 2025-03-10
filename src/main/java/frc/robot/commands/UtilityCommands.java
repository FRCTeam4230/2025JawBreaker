package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class UtilityCommands {

  //for each camera, blink, wait 1 second, andn then turn off
  public static Command flashLimelights(){
    return Commands.runOnce( () -> {
      Constants.limelightCameras.forEach(LimelightHelpers::setLEDMode_ForceBlink);
        }).andThen(Commands.waitSeconds(1)).andThen(Commands.runOnce(() -> {
          Constants.limelightCameras.forEach(LimelightHelpers::setLEDMode_ForceOff);
        }));
  }
}

