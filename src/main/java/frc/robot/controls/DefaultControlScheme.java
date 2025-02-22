package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.TunableController;
import java.util.function.Supplier;

public class DefaultControlScheme implements ControlScheme {
  protected final TunableController controller =
      new TunableController(0)
          .withControllerType(TunableController.TunableControllerType.QUADRATIC);

  @Override
  public Supplier<Double> getFieldX() {
    return () -> -controller.customLeft().getY();
  }

  @Override
  public Supplier<Double> getFieldY() {
    return () -> -controller.customLeft().getX();
  }

  @Override
  public Supplier<Double> getFieldRotation() {
    return () -> -controller.customRight().getX();
  }

  @Override
  public Trigger getL1() {return controller.povDown();}

  @Override
  public Trigger getL2() {return controller.povLeft();}

  @Override
  public Trigger getL3() {return controller.b();}

  @Override
  public Trigger getL4() {
    return controller.povUp();
  }

  @Override
  public Trigger getIntake() {return controller.povRight();}

  @Override
  public Trigger driveToCoralStation() {
    return controller.leftBumper();
  }

  @Override
  public Trigger driveToReef() {
    return controller.rightBumper();
  }

  @Override
  public TunableController getController() {
    return controller;
  }
}
