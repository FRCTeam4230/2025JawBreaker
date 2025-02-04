package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.claw.ClawConstants.GEAR_RATIO;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ClawIOSIMREV extends ClawIOREV {
  private final DCMotorSim motorSimModel;
  private final SparkMaxSim motorSim;

  public ClawIOSIMREV() {
    super();

    DCMotor clawGearBox = DCMotor.getNeo550(1);
    motorSim = new SparkMaxSim(motor, clawGearBox);

    Distance radius = Inches.of(1.5);
    double moi = Pounds.of(8.0).in(Kilograms) * Math.pow(radius.in(Meters), 2);
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createDCMotorSystem(clawGearBox, moi, GEAR_RATIO);
    motorSimModel = new DCMotorSim(linearSystem, clawGearBox);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    super.updateInputs(inputs);
    motorSim.setBusVoltage(RobotController.getBatteryVoltage());

    // In this method, we update our simulation of what our claw is doing
    // First, we set our "inputs" (voltages)
    motorSimModel.setInput(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    motorSimModel.update(0.02);

    // now we update teh sparkmax
    motorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            motorSimModel.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(motorSimModel.getCurrentDrawAmps()));

    // Apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    //        motorSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(GEAR_RATIO));
    motorSim.setVelocity(motorSimModel.getAngularVelocity().times(GEAR_RATIO).in(RadiansPerSecond));
    //        motorSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(GEAR_RATIO));
    motorSim.setPosition(motorSimModel.getAngularPosition().times(GEAR_RATIO).in(Radians));
  }
}
