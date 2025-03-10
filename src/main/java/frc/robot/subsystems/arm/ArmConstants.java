package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public class ArmConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());

  public static final int MOTOR_ID = 32;
  public static final LoggedTunableNumber kP = tunableTable.makeField("kP", 3.0);
  public static final LoggedTunableNumber kI = tunableTable.makeField("kI", 0.0);
  public static final LoggedTunableNumber kD = tunableTable.makeField("kD", 0.0);
  public static final LoggedTunableNumber setpointToleranceRad =
      tunableTable.makeField("setpoint tolerance rad", 0.0);

  public static final Angle ARM_ENCODER_OFFSET_RAD = Rotations.of(4.45 / 6.275);
  public static final int DUTY_CYCLE_ENCODER_PORT = 4;

  public static final LoggedTunableNumber MAX_ARM_PID_VOLTS =
      tunableTable.makeField("max arm pid volts", 0.0);

  public static final int UPPER_LIMIT_SWITCH_DIO_PORT = 3;
  public static final int LOWER_LIMIT_SWITCH_DIO_PORT = 2;

  public static final double MAX_ARM_VOLTS = 0;
  public static final double GEAR_RATIO = 100.0;

  //  public static class Positions {
  //    public static final LoggedTunableNumber PICKUP_POS_RAD =
  //        tunableTable.makeField("pickup pos", 0.0);
  //    public static final LoggedTunableNumber PLACE_POS_RAD =
  //        tunableTable.makeField("place pos", 0.0);
  //  }
}
