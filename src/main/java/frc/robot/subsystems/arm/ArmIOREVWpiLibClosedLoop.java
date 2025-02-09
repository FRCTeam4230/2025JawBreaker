package frc.robot.subsystems.arm;

import com.revrobotics.spark.config.EncoderConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

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

    Voltage calculatedVolts = Volts.of(pidController.calculate(inputs.armAngle.baseUnitMagnitude(), armSetPointAngle.baseUnitMagnitude()));

    if (inputs.upperLimit && calculatedVolts.magnitude() > 0) {
      calculatedVolts = Volts.of(0);
    } else if (inputs.lowerLimit && calculatedVolts.magnitude() < 0) {
      calculatedVolts = Volts.of(0);
    }
    
    //TODO: Do the PID calculations here and then call
    motor.setVoltage(calculatedVolts);
  }

  private void limitAndSetVolts(double volts) {
    if (volts < 0 && lastInputs.lowerLimit) {
      volts = 0;
    }
    if (volts > -.1 && lastInputs.upperLimit) {
      volts = 0;
      //      volts = -0.5;
    }
    volts = MathUtil.clamp(volts, -ArmConstants.MAX_ARM_VOLTS, ArmConstants.MAX_ARM_VOLTS);
    Logger.recordOutput(LOGGING_PREFIX + "attemptedVolts", volts);
    armIO.setVoltage(volts);
  }
}
