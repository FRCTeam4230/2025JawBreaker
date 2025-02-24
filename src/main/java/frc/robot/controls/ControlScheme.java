package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.TunableController;
import java.util.function.Supplier;

public interface ControlScheme {
  public Supplier<Double> getFieldX();

  public Supplier<Double> getFieldY();

  public Supplier<Double> getFieldRotation();

  public Trigger getL1();

  public Trigger getL2();

  public Trigger getL3();

  public Trigger getL4();

  public Trigger getIntake();

  public Trigger score();

  public Trigger driveToCoralStation();

  public Trigger driveToReef();

  // SimButtons
  public default Trigger setLoadGamePiece() {
    return new Trigger(() -> false);
  }

  public default Trigger setUnloadGamePiece() {
    return new Trigger(() -> false);
  }

  public TunableController getController();
}
