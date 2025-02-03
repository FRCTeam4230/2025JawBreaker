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

package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  class ClawIOInputs {
    public boolean leaderConnected = false;
    public boolean encoderConnected = false;
    public boolean hasCoral = false;

    public AngularVelocity velocity = RotationsPerSecond.of(0);

    public Voltage appliedVoltage = Volts.of(0.0);
    public Current supplyCurrent = Amps.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClawIOInputs inputs) {}

  /** Stop in open loop. */
  public default void stop() {}
}
