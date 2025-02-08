package frc.robot.subsystems.elevator;

import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public class ElevatorConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());

  public static final int UPPER_LIMIT_SWITCH_DIO_PORT = 0;
  public static final int LOWER_LIMIT_SWITCH_DIO_PORT = 1;
  public static final int LEADER_MOTOR_ID = 30;
  public static final int LEADER_MOTOR_ENCODER = 4; //
  public static final int FOLLOWER_MOTOR_ID = 31;

  // Converts Rotations to Meters
  public static final double rotationToMeters = 1.0;

  // Converts RPM to MPS
  public static final double rpmToMps = 1.0;

  public static final double elevatorMaxVelocity = 10.0;
  public static final double elevatorMaxAcceleration = 3.0;

  public static final LoggedTunableNumber kP = tunableTable.makeField("kP", 0.1);
  public static final LoggedTunableNumber kI = tunableTable.makeField("kI", 0.0);
  public static final LoggedTunableNumber kD = tunableTable.makeField("kD", 0.0);
}
