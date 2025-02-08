package frc.robot.subsystems.arm;

import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public class ArmConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());

  public static final int MOTOR_ID = 32;
  public static final LoggedTunableNumber kP = tunableTable.makeField("kP", 0.1);
  public static final LoggedTunableNumber kI = tunableTable.makeField("kI", 0.0);
  public static final LoggedTunableNumber kD = tunableTable.makeField("kD", 0.0);
  public static final LoggedTunableNumber setpointToleranceRad =
      tunableTable.makeField("setpoint tolerance rad", 0.0);
  public static final double ARM_ENCODER_OFFSET_RAD = 0;
  public static final int DUTY_CYCLE_ENCODER_PORT = 4;

  public static final LoggedTunableNumber MAX_ARM_PID_VOLTS =
      tunableTable.makeField("max arm pid volts", 0.0);

  public static final double MAX_ARM_VOLTS = 0;
  public static final double GEAR_RATIO = 1.0 / 20.0;

  //  public static class Positions {
  //    public static final LoggedTunableNumber PICKUP_POS_RAD =
  //        tunableTable.makeField("pickup pos", 0.0);
  //    public static final LoggedTunableNumber PLACE_POS_RAD =
  //        tunableTable.makeField("place pos", 0.0);
  //  }
}
