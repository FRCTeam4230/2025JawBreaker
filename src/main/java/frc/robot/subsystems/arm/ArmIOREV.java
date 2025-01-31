package frc.robot.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.units.measure.Voltage;


public class ArmIOREV implements ArmIO {
    private final SparkFlex motor = new SparkFlex(0, SparkLowLevel.MotorType.kBrushless);

    PIDController pidController = new PIDController(0.0, 0.0, 0.0);

    private final RelativeEncoder velocityEncoder = motor.getEncoder();
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);


    public ArmIOREV() {

        EncoderConfig encoderConfig = new EncoderConfig()
                .velocityConversionFactor(1.0);

        EncoderConfig relativeEncoderConfig = new EncoderConfig().positionConversionFactor(2 * Math.PI);
        motor.configure(

            new SparkFlexConfig()
                    .idleMode(SparkBaseConfig.IdleMode.kBrake)
                    .apply(encoderConfig),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

    }

    public void ArmIOInputs(ArmIOInputs inputs) {
        inputs.positionRad = (encoder.get() * Math.PI * 2) - ArmConstants.ARM_ENCODER_OFFSET_RAD;
        inputs.velocityRadPerSec = velocityEncoder.getVelocity();
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.motorSupplyCurrent = motor.getOutputCurrent();
        inputs.motorTemperatureCelsius = motor.getMotorTemperature();
        inputs.motorSensorFault = motor.getFaults().sensor;
        inputs.motorBrownOut = motor.getFaults().other;
        inputs.motorCANID = motor.getDeviceId();
    }


    public void SetVoltage(double volts) {
        volts = MathUtil.clamp(volts, -1.0, 1.0);
        motor.setVoltage(volts);
    }

}
