// package frc.robot.subsystems.counterweight;
//
// import static edu.wpi.first.units.Units.*;
//
// import com.revrobotics.sim.SparkMaxSim;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.Mass;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
//
// public class CounterWeightIOSIM extends CounterWeightIOREV {
//
//  private final SparkMaxSim motorSim;
//  DCMotor dcMotor = DCMotor.getNeoVortex(1);
//
//  private final SingleJointedArmSim armSim;
//  Distance armLength = Inches.of(17);
//  Mass armMass = Pounds.of(2.6);
//  double armMOI = SingleJointedArmSim.estimateMOI(armLength.in(Meters), armMass.in(Kilograms));
//
//  public CounterWeightIOSIM() {
//
//    motorSim = new SparkMaxSim(motor, dcMotor);
//
//    LinearSystem<N2, N1, N2> linearSystem =
//        LinearSystemId.createSingleJointedArmSystem(dcMotor, armMOI, GEAR_RATIO);
//
//    Angle startingAngle = Degrees.of(1);
//    // Initialize the arm simulation:
//    armSim =
//        new SingleJointedArmSim(
//            linearSystem,
//            dcMotor,
//            GEAR_RATIO,
//            armLength.in(Meters),
//            Degrees.of(0).in(Rotations),
//            Degrees.of(180).in(Rotations),
//            false,
//            startingAngle.in(Rotations));
//    motorSim.setPosition(startingAngle.in(Rotations));
//  }
//
//  @Override
//  public void updateInputs(CounterWeightIOInputs inputs) {
//    super.updateInputs(inputs);
//
//    armSim.setInputVoltage(motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
//    armSim.update(0.02);
//
//    double simulatedMotorRPM =
//        Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec() * GEAR_RATIO);
//
//    motorSim.iterate(simulatedMotorRPM, RobotController.getBatteryVoltage(), 0.02);
//  }
// }
