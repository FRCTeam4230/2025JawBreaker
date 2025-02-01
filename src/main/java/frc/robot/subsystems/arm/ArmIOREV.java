package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

public class ArmIOREV implements ArmIO {
  private final SparkFlex motor = new SparkFlex(33, SparkLowLevel.MotorType.kBrushless);
}
