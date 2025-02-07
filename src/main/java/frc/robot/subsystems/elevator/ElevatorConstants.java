package frc.robot.subsystems.elevator;

import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public class ElevatorConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());

  // Converts Rotations to Meters
  public static final double rotationToMeters = Math.PI * 2.0 * 0.08;

  // Converts RPM to MPS
  public static final double rpmToMps = (Math.PI * 2.0 * 0.08) / 60;

  public static final double elevatorMaxVelocity = 10.0;
  public static final double elevatorMaxAcceleration = 3.0;
  public static final double GEAR_RATIO = 12.0;

  public static final LoggedTunableNumber kP = tunableTable.makeField("kP", 0.1);
  public static final LoggedTunableNumber kI = tunableTable.makeField("kI", 0.0);
  public static final LoggedTunableNumber kD = tunableTable.makeField("kD", 0.0);
}
