package frc.robot.subsystems.elevator;

import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public class ElevatorConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());

  // Converts Rotations to Meters
  public static final double rotationToMeters = 1.0;

  // Converts RPM to MPS
  public static final double rpmToMps = 1.0;

  public static final double elevatorMaxVelocity = 10.0;
  public static final double elevatorMaxAcceleration = 3.0;

  public static final LoggedTunableNumber kP = tunableTable.makeField("kP", 1);
  public static final LoggedTunableNumber kI = tunableTable.makeField("kI", 0.0);
  public static final LoggedTunableNumber kD = tunableTable.makeField("kD", 0.5);
}
