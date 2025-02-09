package frc.robot.subsystems.arm;

import com.revrobotics.spark.config.EncoderConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;

public class ArmIOREVWpiLibClosedLoop extends ArmIOREV {

  private final PIDController pidController = new PIDController(0, 0, 0);

  public ArmIOREVWpiLibClosedLoop() {
    super();
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
    //TODO: store the SetPoint, this is the angle to the PID contorller
  }

  /**
   * @param inputs
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    super.updateInputs(inputs);

    //TODO: Do the PID calculations here and then call
    motor.setVoltage(0);
  }
}
