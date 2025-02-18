package frc.robot.subsystems.counterweight;

import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public class CounterWeightConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());

  public static final int MOTOR_ID = 36;
  protected static final double GEAR_RATIO = 125.0;

  public static final LoggedTunableNumber kP = tunableTable.makeField("kP", 0.3);
  public static final LoggedTunableNumber kI = tunableTable.makeField("kI", 0.0);
  public static final LoggedTunableNumber kD = tunableTable.makeField("kD", 0.0);
}
