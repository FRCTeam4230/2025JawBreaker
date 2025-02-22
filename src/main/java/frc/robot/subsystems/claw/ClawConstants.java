package frc.robot.subsystems.claw;

import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public class ClawConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass()); // not tuned yet
  public static final int clawMotorID = 33;
  public static final int beamBreakDIOPort = 7;

  public static final LoggedTunableNumber INTAKE_VOLTAGE =
      tunableTable.makeField("intake volts", 6);
  public static final LoggedTunableNumber HOLD_VOLTAGE = tunableTable.makeField("hold volts", 0.5);

  public static final double GEAR_RATIO = (36.0 / 24) * 5.0; // 36 teeth to 24 teeth, 5:1 gear ratio
  public static final double CLOSED_LOOP_RAMP_RATE = 0.2;
  public static final double OPEN_LOOP_RAMP_RATE = 0.2;
}
