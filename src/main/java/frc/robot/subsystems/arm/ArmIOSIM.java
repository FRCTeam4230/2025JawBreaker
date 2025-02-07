// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of the arm subsystem. This class extends ArmIOREV to provide a
 * physics-based simulation of the arm mechanism using WPILib's simulation classes.
 *
 * <p>The simulation models: - Dual Kraken X60 FOC motors - Realistic arm physics including gravity
 * and moment of inertia - Position and velocity feedback through simulated encoders - Battery
 * voltage effects - Motion limits (0° to 180°)
 */
public class ArmIOSIM extends ArmIOREV {

  /** Physics simulation model for the arm mechanism */
  private final SingleJointedArmSim motorSimModel;

  /** Simulation state for the motor */
  private final SparkFlexSim motorSim;

  private final SparkRelativeEncoderSim velocityEncoderSim;
  /** Simulation state for the CANcoder */
  private final DutyCycleEncoderSim encoderSim;

  /** Constructs a new ArmIOSIM instance. */
  public ArmIOSIM() {
    super(); // Initialize hardware interface components

    // Get simulation states for all hardware
    DCMotor gearbox = DCMotor.getNEO(2);
    motorSim = new SparkFlexSim(motor, gearbox);
    velocityEncoderSim = new SparkRelativeEncoderSim(motor);
    encoderSim = new DutyCycleEncoderSim(ArmConstants.DUTY_CYCLE_ENCODER_PORT);

    // encoderSim = new SparkRelativeEncoderSim(motor);

    // Configure dual Kraken X60 FOC motors
    // DCMotor motor = DCMotor.getKrakenX60Foc(2);

    // Define arm physical properties
    Distance armLength = Inches.of(23.75);
    Mass armMass = Pounds.of(4.84);

    // Calculate moment of inertia using WPILib helper
    double armMOI = SingleJointedArmSim.estimateMOI(armLength.in(Meters), armMass.in(Kilograms));

    // Create arm physics model
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(2), armMOI, GEAR_RATIO);

    // Initialize arm simulation

    motorSimModel =
        new SingleJointedArmSim(
            linearSystem,
            gearbox,
            GEAR_RATIO,
            armLength.in(Meters),
            Degrees.of(0).in(Radians), // Lower limit (0°)
            Degrees.of(180).in(Radians), // Upper limit (180)
            false, // Enable gravity simulation
            Degrees.of(90).in(Radians)); // Start at 90°
  }

  /**
   * Updates the simulation model and all simulated sensor inputs.
   *
   * @param inputs The ArmIOInputs object to update with simulated values
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update base class inputs first
    super.updateInputs(inputs);

    // Simulate battery voltage effects on all devices
    motorSim.setBusVoltage(RobotController.getBatteryVoltage());
    motorSimModel.setInput(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Update physics simulation
    motorSimModel.update(0.02); // Simulate 20ms timestep (50Hz)

    motorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            motorSimModel.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02);
    velocityEncoderSim.iterate(motorSim.getVelocity(), 0.02);

    // Get position and velocity from physics simulation
    Angle position = Radians.of(motorSimModel.getAngleRads());
    AngularVelocity velocity = RadiansPerSecond.of(motorSimModel.getVelocityRadPerSec());

    // Update simulated motor encoder readings (accounts for gear ratio)
    motorSim.setPosition(position.times(GEAR_RATIO).in(Degrees));
    motorSim.setVelocity(velocity.times(GEAR_RATIO).in(DegreesPerSecond));

    // Update simulated CANcoder readings (direct angle measurement).

    encoderSim.set(velocity.in(DegreesPerSecond));
  }
}
