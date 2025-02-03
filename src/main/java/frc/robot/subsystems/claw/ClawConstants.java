package frc.robot.subsystems.claw;

import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public class ClawConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass()); // not tuned yet
  public static final int clawMotorID = 0;

  public static final LoggedTunableNumber INTAKE_VOLTAGE =
      tunableTable.makeField("intake volts", 6);

  public static final double CLOSED_LOOP_RAMP_RATE = 0.2;
  public static final double OPEN_LOOP_RAMP_RATE = 0.2;
}
