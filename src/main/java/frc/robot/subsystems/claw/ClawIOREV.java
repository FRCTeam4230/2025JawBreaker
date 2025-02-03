package frc.robot.subsystems.claw;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj.I2C;

public class ClawIOREV implements ClawIO {
  private final SparkFlex motor = new SparkFlex(33, SparkLowLevel.MotorType.kBrushless);

  private final ColorSensorV3 colorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);

  public void rollerSpeed(double speed) {
    motor.set(speed);
  }

  public boolean hasCoral() {
    return colorSensorV3.getProximity() > 1;
  }

  public void stop() {
    motor.set(0);
  }
}
