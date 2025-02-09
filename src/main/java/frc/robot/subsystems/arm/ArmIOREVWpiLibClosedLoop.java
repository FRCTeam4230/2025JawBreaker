package frc.robot.subsystems.arm;

import com.revrobotics.spark.config.EncoderConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ArmIOREVWpiLibClosedLoop extends ArmIOREV {

  private final PIDController pidController;

  public ArmIOREVWpiLibClosedLoop() {
    super();
    pidController = new PIDController(ArmConstants.kP.get(), ArmConstants.kI.get(), ArmConstants.kD.get());
  }

  /**
   * @return EncoderConfig
   */
  @Override
  protected EncoderConfig getEncoderConfig() {
    return super.getEncoderConfig();
  }


  @Override
  public void reconfigurePID() {
    pidController.setPID(ArmConstants.kP.get(), ArmConstants.kI.get(), ArmConstants.kD.get());
    pidController.setTolerance(ArmConstants.setpointToleranceRad.get());
  }

  @Override
  public void setPosition(Angle position) {
    this.armSetPointAngle = position;
  }

  /**
   * @param inputs
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    super.updateInputs(inputs);

    //dont' know if we can actually override the encocer velocity with the absolute..



    //TODO: Do the PID calculations here and then call
    Voltage calculatedVolts = Volts.of(pidController.calculate(inputs.armAngle.baseUnitMagnitude(), armSetPointAngle.baseUnitMagnitude()));

    //if we are upa dn trying to go up more, stop, or if we are going down and are at the bottom, stop
    //TODO: these bool checks may be backwards due to inversion
    if (inputs.upperLimit && calculatedVolts.gt(ZERO_VOLTS) ||
        inputs.lowerLimit && calculatedVolts.lt(ZERO_VOLTS)){

      calculatedVolts = ZERO_VOLTS;
    }else {
      calculatedVolts = Volts.of(MathUtil.clamp(calculatedVolts.baseUnitMagnitude(), ArmConstants.MIN_ARM_VOLTS, ArmConstants.MAX_ARM_VOLTS));
    }

    inputs.attemptedVoltage = calculatedVolts;
    motor.setVoltage(calculatedVolts);
  }

}
