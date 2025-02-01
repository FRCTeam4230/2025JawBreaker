// Copyright FRC 5712
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public boolean motorConnected = false;
    public boolean encoderConnected = false;

    public Angle motorPosition = Rotations.of(0);
    public Angle encoderPosition = Rotations.of(0);

    public AngularVelocity motorVelocity = RotationsPerSecond.of(0);

    public AngularVelocity encoderVelocity = RotationsPerSecond.of(0);

    public Voltage appliedVoltage = Volts.of(0.0);
    public Current motorStatorCurrent = Amps.of(0);
    public Current motorSupplyCurrent = Amps.of(0);
    public Angle armAngle = Rotations.of(0);

    public Temperature motorTemperatureCelsius = Celsius.of(0.0);

    public int motorCANID = -1;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ArmIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  default void setPosition(Angle angle) {}

  /** Stop in open loop. */
  default void stop() {}
}
