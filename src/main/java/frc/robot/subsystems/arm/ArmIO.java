// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public boolean leaderConnected = false;
    public boolean encoderConnected = false;

    public Angle leaderPosition = Rotations.of(0);
    public Angle leaderRotorPosition = Rotations.of(0);

    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity leaderRotorVelocity = RotationsPerSecond.of(0);

    public Voltage appliedVoltage = Volts.of(0.0);
    public Current leaderStatorCurrent = Amps.of(0);
    public Current leaderSupplyCurrent = Amps.of(0);
    public Angle armAngle = Rotations.of(0);
    public Angle armSetPointAngle = Rotations.of(0);

    public Temperature motorTemperatureCelsius = Celsius.of(0.0);

    public int motorCANID = ArmConstants.MOTOR_ID;
    public double motorPositionFactor;

    public boolean lowerLimit = false;
    public boolean upperLimit = false;

    public Voltage attemptedVoltage = Volts.of(0);
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ArmIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  default void setPosition(Angle angle) {}

  /** Stop in open loop. */
  default void stop() {}

  default void reconfigurePID() {}

  default void resetEncoder() {}
}
