package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;


public class ReefAlignCommand extends Command {
  private final Drive drivetrain;
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;

  private final NetworkTable limelightTable;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  // these are only for blue side - use alliance utils
  private static final Set<Integer> REEF_TAG_IDS = Set.of(17, 18, 19, 20, 21, 22);
  private static final Set<Integer> REFILL_TAG_IDS = Set.of(12, 13);

  public enum StationType {
    REEF,
    REFILL
  }

  // how far from the tag to stop in metres
  private static final double REEF_DISTANCE = 0.3;
  private static final double REFILL_DISTANCE = 0.3;

  private final StationType stationType;
  private final boolean alignLeft;

  public ReefAlignCommand(Drive drive, StationType type, boolean alignLeftBool) {
    drivetrain = drive;
    stationType = type;
    alignLeft = alignLeftBool;

    limelightTable =
        NetworkTableInstance.getDefault()
            .getTable("limelight-front"); // is this supposed to be limelight-front/limelight-back?

    // TODO: change these
    xController = new PIDController(1.0, 0.0, 0.0);
    yController = new PIDController(1.0, 0.0, 0.0);
    rotationController = new PIDController(1.0, 0.0, 0.0);

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // tv checks for valid target
    double tv = limelightTable.getEntry("tv").getDouble(0);
    // gets the id of the "primary in-view" ? hopefully that only returns the one we are looking at
    // from front cam
    double tid = limelightTable.getEntry("tid").getDouble(-1);

    if (tv < 1 || !isValidTag((int) tid)) {
      stopRobot();
      return;
    }

    double[] targetpose =
        limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

    if (targetpose.length < 6) {
      stopRobot();
      return;
    }

    double x = targetpose[0];
    double y = targetpose[1];
    double rot = targetpose[5];

    double desiredX =
        x
            + (alignLeft
                ? 0.1
                : -0.1); // offset in metres to move left or right according to which side we are
    // aligning to
    double desiredY = y - (stationType == StationType.REEF ? REEF_DISTANCE : REFILL_DISTANCE);
    double desiredRot = 0;

    // if rotations is in radians this should rotate it 180 degrees?
    if (stationType == StationType.REFILL && x < 0) {
      desiredRot = Math.PI; // 180 degrees
    }

    // pid calc

    double xVelocity = xController.calculate(x, desiredX);
    double yVelocity = yController.calculate(y, desiredY);
    double rotationalRate = rotationController.calculate(rot, desiredRot);

    // clamped but can be changed
    final double clampedXVelocity = MathUtil.clamp(xVelocity, -1.0, 1.0);
    final double clampedYVelocity = MathUtil.clamp(yVelocity, -1.0, 1.0);
    final double clampedRotationalRate = MathUtil.clamp(rotationalRate, -1.0, 1.0);

    drivetrain.applyRequest(
        () ->
            drivetrain
                .getSetpointGenerator()
                .withVelocityX(MaxSpeed.times(clampedXVelocity))
                .withVelocityY(MaxSpeed.times(clampedYVelocity))
                .withRotationalRate(Constants.MaxAngularRate.times(clampedRotationalRate)));
  }

  private void stopRobot() {
    drivetrain.applyRequest(
        () ->
            drivetrain
                .getSetpointGenerator()
                .withVelocityX(MaxSpeed.times(0))
                .withVelocityY(MaxSpeed.times(0))
                .withRotationalRate(Constants.MaxAngularRate.times(0)));
  }

  private boolean isValidTag(int tagId) {
    return switch (stationType) {
      case REEF -> REEF_TAG_IDS.contains(tagId);
      case REFILL -> REFILL_TAG_IDS.contains(tagId);
    };
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    stopRobot();
  }

  // could we use different pipelines using a map, if going for top using pipeline 3, if going for
  // middle pipeline 2, etc
}
