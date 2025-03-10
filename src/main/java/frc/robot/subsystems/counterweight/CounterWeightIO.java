// package frc.robot.subsystems.counterweight;
//
// import static edu.wpi.first.units.Units.*;
//
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Voltage;
// import org.littletonrobotics.junction.AutoLog;
//
// public interface CounterWeightIO {
//  @AutoLog
//  class CounterWeightIOInputs {
//    public boolean motorConnected = false;
//    public boolean encoderConnected = false;
//
//    public boolean climberExtended = false;
//    public boolean climberRetracted = false;
//
//    public Angle motorPosition = Rotations.of(0);
//
//    public AngularVelocity motorVelocity = RotationsPerSecond.of(0);
//
//    public Voltage appliedVoltage = Volts.of(0.0);
//    public Current motorStatorCurrent = Amps.of(0);
//    public Current motorSupplyCurrent = Amps.of(0);
//
//    public Angle setpoint = Rotations.of(0);
//  }
//
//  default void updateInputs(CounterWeightIOInputs inputs) {}
//
//  default void setPosition(Voltage volts) {}
//
//  default void stop() {}
// }
