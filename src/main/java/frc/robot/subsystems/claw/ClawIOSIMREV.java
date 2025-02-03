package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.claw.ClawConstants.GEAR_RATIO;

import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClawIOSIMREV extends ClawIOREV {
  private final DCMotorSim motorSimModel;

  private final SparkFlexSim motorSim;

  public ClawIOSIMREV() {
    super();

    DCMotor dcMotor = DCMotor.getNEO(1);
    motorSim = new SparkFlexSim(motor, dcMotor);

    Distance radius = Inches.of(1.5);
    double moi = Pounds.of(8.0).in(Kilograms) * Math.pow(radius.in(Meters), 2);
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createDCMotorSystem(dcMotor, moi, GEAR_RATIO);
    motorSimModel = new DCMotorSim(linearSystem, dcMotor);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    super.updateInputs(inputs);
    motorSim.setBusVoltage(RobotController.getBatteryVoltage());

    double motorVoltage = motorSim.getAppliedOutput();
    if (motorVoltage == 0) {
      //            motorVoltage =
      //
      // motorSimModel.getAngularVelocity().times(GEAR_RATIO).in(RadiansPerSecond)
      //                            / motorSimModel.getA
      motorVoltage =
          motorSimModel.getAngularVelocity().times(GEAR_RATIO).in(RadiansPerSecond)
              / motorSimModel.getAngularAcceleration().in(RadiansPerSecondPerSecond);
      // add friction, this could be pulled outside the if to always apply friction
      if (motorVoltage > 0.2) {
        motorVoltage -= 0.2;
      } else if (motorVoltage < -0.2) {
        motorVoltage += 0.2;
      } else {
        motorVoltage = 0.0;
      }
    }

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage);
    motorSimModel.update(0.020); // assume 20 ms loop time

    // Apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    //        motorSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(GEAR_RATIO));
    motorSim.setVelocity(motorSimModel.getAngularVelocity().times(GEAR_RATIO).in(RadiansPerSecond));
    //        motorSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(GEAR_RATIO));
    motorSim.setPosition(motorSimModel.getAngularPosition().times(GEAR_RATIO).in(Radians));
  }
}
