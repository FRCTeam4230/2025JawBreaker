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
    public Angle setpoint = Rotations.of(0);

    public boolean upperlimit = false;
    public boolean lowerlimit = false;

    public Temperature motorTemperatureCelsius = Celsius.of(0.0);

    public int motorCANID = -1;

    public double feedbackSensorValue;
    public double motorPositionFactor;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ArmIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  default void setPosition(Angle angle) {}

  /** Stop in open loop. */
  default void stop() {}

  default void reconfigurePID() {}
}
