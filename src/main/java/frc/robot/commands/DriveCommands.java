// This file is based on code from team 6328 Mechanical Advantage
// See here for the original source:
// https://github.com/Mechanical-Advantage/RobotCode2024/blob/c0c6d11547769f6dc5f304d5c18c9b51086a691b/src/main/java/org/littletonrobotics/frc2024/commands/WheelRadiusCharacterization.java

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.requests.SwerveSetpointGen;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import org.littletonrobotics.junction.Logger;

public class DriveCommands extends Command {

  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());

  public static final LoggedTunableNumber kP = tunableTable.makeField("kP", 1.2);
  public static final LoggedTunableNumber kI = tunableTable.makeField("kI", 0.0);
  public static final LoggedTunableNumber kD = tunableTable.makeField("kD", 0.0);

  // private static PhoenixPIDController translationController =
  private static PIDController translationController =
      new PIDController(kP.get(), kI.get(), kD.get());

  private static PIDController rotationController = new PIDController(5, 0, 0);

  static {
    rotationController.enableContinuousInput(-0.5, 0.5);
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(0.1);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
    SwerveRequest.RobotCentric req =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withVelocityX(0)
            .withVelocityY(0);

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(6);
                  drive.setControl(req.withRotationalRate(speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getDrivePositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      Angle[] positions = drive.getDrivePositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < Constants.PP_CONFIG.numModules; i++) {
                        wheelDelta +=
                            Math.abs(
                                    positions[i].minus(state.positions[i]).baseUnitMagnitude()
                                        / Constants.SWERVE_MODULE_CONSTANTS.DriveMotorGearRatio)
                                / Constants.PP_CONFIG.numModules;
                      }
                      double wheelRadius =
                          (state.gyroDelta * Constants.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                      System.out.println(
                          "\tDrive base radius: "
                              + formatter.format(Constants.DRIVE_BASE_RADIUS)
                              + " meters");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    Angle[] positions = new Angle[Constants.PP_CONFIG.numModules];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  public static void driveToPointMA(Pose2d target, Drive drive) {
    driveToPointMA(target, drive, Constants.robotScoringOffset, false);
  }

  public static void driveToPointMA(Pose2d target, Drive drive, boolean isBackOfRobot) {
    driveToPointMA(target, drive, Constants.robotScoringOffset, isBackOfRobot);
  }

  public static void driveToPointMA(
      Pose2d target, Drive drive, Distance offset, boolean isBackOfRobot) {
    Pose2d newTarget = getDriveTarget(drive.getPose(), target, offset, isBackOfRobot);
    driveToPoint(newTarget, drive);
  }

  /** Get drive target. */
  private static Pose2d getDriveTarget(
      Pose2d robot, Pose2d goal, Distance robotOffset, boolean isBackOfRobot) {

    if (isBackOfRobot) {
      goal = GeomUtil.flipRotation(goal);
    }

    // Final line up
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());

    double shiftXT =
        MathUtil.clamp(
            (yDistance / (FieldConstants.Reef.faceLength.in(Meters) * 2))
                + ((xDistance - 0.3) / (FieldConstants.Reef.faceLength.in(Meters) * 3)),
            0.0,
            1.0);
    double shiftYT =
        MathUtil.clamp(offset.getX() / FieldConstants.Reef.faceLength.in(Meters), 0.0, 1.0);

    Pose2d goalPose =
        goal.transformBy(
            GeomUtil.toTransform2d(
                -shiftXT * Constants.maxDistanceReefLineup.in(Meters),
                Math.copySign(
                    shiftYT * Constants.maxDistanceReefLineup.in(Meters) * 0.8, offset.getY())));

    if (isBackOfRobot) {
      goalPose = GeomUtil.flipRotation(goalPose);
    }

    return goalPose;
  }

  public static void driveToPoint(Pose2d target, Drive drive) {
    driveToPoint(target, drive, Constants.robotScoringOffset);
  }

  public static void driveToPoint(Pose2d target, Drive drive, Distance offset) {
    Pose2d current = drive.getPose();
    double currentTimestamp = drive.getCurrentTimestamp();
    // double pidX = translationController.calculate(current.getX(), target.getX(),
    // currentTimestamp);
    // double pidY = translationController.calculate(current.getY(), target.getY(),
    // currentTimestamp);
    double pidX = translationController.calculate(current.getX(), target.getX());
    double pidY = translationController.calculate(current.getY(), target.getY());
    double pidRot =
        rotationController.calculate(
            drive.getRotation().getRotations(), target.getRotation().getRotations());
    // drive.getCurrentTimestamp());
    ChassisSpeeds speeds = new ChassisSpeeds(pidX, pidY, Rotations.of(pidRot).in(Radians));

    SwerveSetpointGen setpointGenerator = drive.getSetpointGenerator();

    setpointGenerator
        .withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond);
    drive.setControl(setpointGenerator);
    Logger.recordOutput("Drive/TargetPose", target);
  }

  public static void reconfigurePID() {
    translationController.setPID(kP.get(), kI.get(), kD.get());
  }
}
