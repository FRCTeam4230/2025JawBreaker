// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public boolean encoderConnected = false;

    public Distance leaderPosition = Meters.of(0);
    public Distance leaderRotorPosition = Meters.of(0);
    public Distance encoderPosition = Meters.of(0);

    public LinearVelocity leaderVelocity = MetersPerSecond.of(0);
    public LinearVelocity leaderRotorVelocity = MetersPerSecond.of(0);
    public LinearVelocity encoderVelocity = MetersPerSecond.of(0);

    public Voltage appliedVoltage = Volts.of(0.0);
    public Current leaderStatorCurrent = Amps.of(0);
    public Current followerStatorCurrent = Amps.of(0);
    public Current leaderSupplyCurrent = Amps.of(0);
    public Current followerSupplyCurrent = Amps.of(0);

    public Distance elevatorDistance = Inches.of(0);
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  default void setDistance(Distance distance) {}

  /** Stop in open loop. */
  default void stop() {}
}
