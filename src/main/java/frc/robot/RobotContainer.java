package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOREV;
import frc.robot.subsystems.arm.ArmIOREVSIM;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawIOREV;
import frc.robot.subsystems.claw.ClawIOSIMREV;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOREV;
import frc.robot.subsystems.climber.ClimberIOSIM;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOREV;
import frc.robot.subsystems.elevator.ElevatorIOSIMREV;
import frc.robot.subsystems.lights.LightsCTRE;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  // private final ControlScheme controlScheme;

  private final TunableController primaryController =
      new TunableController(0).withControllerType(TunableControllerType.LINEAR);
  private final TunableController secondController =
      new TunableController(1).withControllerType(TunableControllerType.LINEAR);

  private final LoggedDashboardChooser<Command> autoChooser;

  public final Drive drivetrain;
  // CTRE Default Drive Request
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.05))
          .withRotationalDeadband(MaxAngularRate.times(0.05)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Elevator elevator;
  private final Arm arm;
  private final Claw claw;
  private final Climber climber;
  private final LightsCTRE lights;
  //  private final CounterWeight counterWeight;

  private final ScoringCommands scoreCommands;
  private final DriveCommands.AprilTagToBranch aprilTagToBranch;

  @AutoLogOutput
  Pose2d reefBranch =
      FieldConstants.Reef.branchPositions.get(0).get(FieldConstants.ReefHeight.L4).toPose2d();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public RobotContainer() {

    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = new Drive(currentDriveTrain);

        new Vision(
            drivetrain::addVisionData,
            new VisionIOLimelight("limelight-fr", drivetrain::getVisionParameters),
            new VisionIOLimelight("limelight-fl", drivetrain::getVisionParameters));

        elevator = new Elevator(new ElevatorIOREV() {});
        arm = new Arm(new ArmIOREV() {});
        claw = new Claw(new ClawIOREV() {});

        climber = new Climber(new ClimberIOREV() {});

        lights = new LightsCTRE();
        //        counterWeight = new CounterWeight(new CounterWeightIOREV());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new Drive(currentDriveTrain);

        //        new Vision(
        //            drivetrain::addVisionData,
        //            new VisionIOPhotonVisionSIM(
        //                "Front Camera",
        //                new Transform3d(
        //                    new Translation3d(0.2, 0.0, 0.8),
        //                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(0))),
        //                drivetrain::getVisionParameters),
        //            new VisionIOPhotonVisionSIM(
        //                "Back Camera",
        //                new Transform3d(
        //                    new Translation3d(-0.2, 0.0, 0.8),
        //                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(180))),
        //                drivetrain::getVisionParameters),
        //            new VisionIOPhotonVisionSIM(
        //                "Left Camera",
        //                new Transform3d(
        //                    new Translation3d(0.0, 0.2, 0.8),
        //                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(90))),
        //                drivetrain::getVisionParameters),
        //            new VisionIOPhotonVisionSIM(
        //                "Right Camera",
        //                new Transform3d(
        //                    new Translation3d(0.0, -0.2, 0.8),
        //                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(-90))),
        //                drivetrain::getVisionParameters));

        elevator = new Elevator(new ElevatorIOSIMREV());
        arm = new Arm(new ArmIOREVSIM());
        claw = new Claw(new ClawIOSIMREV());
        climber = new Climber(new ClimberIOSIM());
        lights = new LightsCTRE();
        //        counterWeight = new CounterWeight(new CounterWeightIOSIM());

        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain = new Drive(new DriveIO() {});

        new Vision(
            drivetrain::addVisionData,
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {});

        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIOREV() {});
        claw = new Claw(new ClawIOREV() {});
        climber = new Climber(new ClimberIOREV() {});
        lights = new LightsCTRE();
        //        counterWeight = new CounterWeight(new CounterWeightIOREV());
        break;
    }

    aprilTagToBranch = new DriveCommands.AprilTagToBranch(drivetrain);
    scoreCommands = new ScoringCommands(elevator, arm, claw);

    //    NamedCommands.registerCommand("scoreCoral", scoreCommands.score());
    //    NamedCommands.registerCommand("intake", scoreCommands.intakeCoral());
    //    NamedCommands.registerCommand("topLevel", scoreCommands.topLevel());
    new EventTrigger("topLevel").onTrue(scoreCommands.topLevel());
    new EventTrigger("scoreCoral").onTrue(scoreCommands.score());
    new EventTrigger("waitForCoral")
        .onTrue(Commands.waitUntil(() -> elevator.hasCoral() || claw.hasCoral()));
    new EventTrigger("intakeCoral").onTrue(scoreCommands.intakeCoral());
    NamedCommands.registerCommand("waitForCoral", Commands.waitUntil(elevator::hasCoral));

    List<PathPlannerPath> originalPaths = null;
    try {
      originalPaths =
          PathPlannerAuto.getPathGroupFromAutoFile("Copy of Right Side Full Auto Attempt");
    } catch (IOException | ParseException e) {
      throw new RuntimeException(e);
    }
    List<PathPlannerPath> mirroredPaths = new ArrayList<>();
    mirroredPaths.add(originalPaths.get(0).mirrorPath());
    Command mirroredPathCommand = AutoBuilder.followPath(mirroredPaths.get(0));

    //      List<PathPlannerPath> originalPaths = PathPlannerAuto.getPathGroupFromAutoFile("Left
    // Side from Right Copy Attempt");
    //      List<PathPlannerPath> mirroredPaths = new ArrayList<>();
    //      originalPaths.forEach(path -> {
    //        PathPlannerPath mirroredPath = path.mirrorPath();
    //        mirroredPaths.add(mirroredPath);
    //      });

    for (PathPlannerPath path : originalPaths) {
      PathPlannerPath mirroredPath = path.mirrorPath();
      mirroredPaths.add(mirroredPath);
    }

    for (int i = 1; i < mirroredPaths.size(); i++) {
      mirroredPathCommand =
          mirroredPathCommand.andThen(AutoBuilder.followPath(mirroredPaths.get(i)));
    }

    // Set up auto routines
    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Choices", AutoBuilder.buildAutoChooser()); // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drivetrain));
    autoChooser.addOption("Mirrored Path", mirroredPathCommand);

    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drivetrain
                    .getSetpointGenerator()
                    .withVelocityX(
                        MaxSpeed.times(
                            primaryController.customRight().getY()
                                * -1)) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            primaryController.customRight().getX()
                                * -1)) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            primaryController.customLeft().getX()
                                * -1)))); // Drive counterclockwise with
    // negative X (left)

    //    primaryController.back().onTrue(Commands.runOnce(() ->
    // drivetrain.resetPose(Pose2d.kZero)));
    //    joystick
    //        .b()
    //        .whileTrue(
    //            drivetrain.applyRequest(
    //                () ->
    //                    point.withModuleDirection(
    //                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Custom Swerve Request that use PathPlanner Setpoint Generator. Tuning NEEDED. Instructions
    // can be found here
    // https://hemlock5712.github.io/Swerve-Setup/talonfx-swerve-tuning.html
    //    SwerveSetpointGen setpointGen =
    //        new SwerveSetpointGen(
    //                drivetrain.getChassisSpeeds(),
    //                drivetrain.getModuleStates(),
    //                drivetrain::getRotation)
    //            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    //    joystick
    //        .x()
    //        .whileTrue(
    //            drivetrain.applyRequest(
    //                () ->
    //                    setpointGen
    //                        .withVelocityX(MaxSpeed.times(-joystick.getLeftY()))
    //                        .withVelocityY(MaxSpeed.times(-joystick.getLeftX()))
    //
    // .withRotationalRate(Constants.MaxAngularRate.times(-joystick.getRightX()))
    //
    // .withOperatorForwardDirection(drivetrain.getOperatorForwardDirection())));

    //    joystick
    //        .x()
    //        .whileTrue(
    //            drivetrain.applyRequest(
    //                () ->
    //                    setpointGen
    //                        .withVelocityX(
    //                            MaxSpeed.times(
    //                                -joystick.getLeftY())) // Drive forward with negative Y
    // (forward)
    //                        .withVelocityY(MaxSpeed.times(-joystick.getLeftX()))
    //
    // .withRotationalRate(Constants.MaxAngularRate.times(-joystick.getRightX()))
    //
    // .withOperatorForwardDirection(drivetrain.getOperatorForwardDirection())));

    // Custom Swerve Request that use ProfiledFieldCentricFacingAngle. Allows you to face specific
    // direction while driving
    //    ProfiledFieldCentricFacingAngle driveFacingAngle =
    //        new ProfiledFieldCentricFacingAngle(
    //                new TrapezoidProfile.Constraints(
    //                    MaxAngularRate.baseUnitMagnitude(),
    //                    MaxAngularRate.div(0.25).baseUnitMagnitude()))
    //            .withDeadband(MaxSpeed.times(0.05))
    //            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    //    // Set PID for ProfiledFieldCentricFacingAngle
    //    driveFacingAngle.HeadingController.setPID(7, 0, 0);

    //    joystick
    //        .start()
    //        .whileTrue(
    //            drivetrain
    //                .runOnce(() -> driveFacingAngle.resetProfile(drivetrain.getRotation()))
    //                .andThen(
    //                    drivetrain.applyRequest(
    //                        () ->
    //                            driveFacingAngle
    //                                .withVelocityX(
    //                                    MaxSpeed.times(
    //                                        -joystick
    //                                            .getLeftY())) // Drive forward with negative Y
    // (forward)
    //                                .withVelocityY(MaxSpeed.times(-joystick.getLeftX()))
    //                                .withTargetDirection(
    //                                    new Rotation2d(
    //                                        -joystick.getRightY(), -joystick.getRightX())))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    //    joystick
    //        .back()
    //        .and(joystick.y())
    //        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //    joystick
    //        .back()
    //        .and(joystick.x())
    //        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    //
    //    joystick
    //        .start()
    //        .and(joystick.y())
    //        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //
    //    joystick
    //        .start()
    //        .and(joystick.x())
    //        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // testJoystick.back().onTrue(arm.reconfigPID());

    /****** ASSISTED DRIVE **** /
     *
     */
    // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    // controlScheme.getL1().onTrue(superstructure.CORAL_PLACE_L1());
    // controlScheme.getL2().onTrue(superstructure.CORAL_PLACE_L2());
    // controlScheme.getL3().onTrue(superstructure.CORAL_PLACE_L3());
    // controlScheme.getL4().onTrue(superstructure.CORAL_PLACE_L4());

    /*
      controlScheme
          .getL4()
          .onTrue(
              Commands.runOnce(
                  () -> SmartController.targetReefHeight = FieldConstants.ReefHeight.L4));
      controlScheme
          .getL3()
          .onTrue(
              Commands.runOnce(
                  () -> SmartController.targetReefHeight = FieldConstants.ReefHeight.L3));
      controlScheme
          .getL2()
          .onTrue(
              Commands.runOnce(
                  () -> SmartController.targetReefHeight = FieldConstants.ReefHeight.L2));
      controlScheme
          .getL1()
          .onTrue(
              Commands.runOnce(
                  () -> SmartController.targetReefHeight = FieldConstants.ReefHeight.L1));

      coralWrist.hasGamePiece().onFalse(superstructure.INTAKE()).onTrue(coralWrist.FRONT());

      coralWrist
          .hasGamePiece()
          .and(() -> SmartController.targetReefHeight == FieldConstants.ReefHeight.L2)
          .onTrue(superstructure.TRANSIT());
      coralWrist
          .hasGamePiece()
          .and(() -> SmartController.targetReefHeight != FieldConstants.ReefHeight.L2)
          .onTrue(superstructure.CORAL_PLACE_L3());

      controlScheme.setLoadGamePiece().onTrue(coralWrist.gamePieceLoaded());
      controlScheme.setUnloadGamePiece().onTrue(coralWrist.gamePieceUnloaded());

      controlScheme
          .driveToCoralStation()
          .whileTrue(
              Commands.run(
                  () ->
                      DriveCommands.driveToPointMA(
                          FieldConstants.CoralStation.leftCenterFace.transformBy(
                              new Transform2d(
                                  new Translation2d(Constants.robotScoringOffset, Inches.of(1.8)),
                                  Rotation2d.kZero)),
                          drivetrain,
                          true),
                  drivetrain));

      Trigger autoReefHeight =
          new Trigger(
              () ->
                  drivetrain.getPose().getTranslation().getDistance(FieldConstants.Reef.center)
                      < 3.0);

      autoReefHeight.whileTrue(
          Commands.runOnce(
              () -> {
                switch (SmartController.targetReefHeight) {
                  case L1:
                    superstructure.CORAL_PLACE_L1().schedule();
                    break;
                  case L2:
                    superstructure.CORAL_PLACE_L2().schedule();
                    break;
                  case L3:
                    superstructure.CORAL_PLACE_L3().schedule();
                    break;
                  case L4:
                    superstructure.CORAL_PLACE_L4().schedule();
                    break;
                  default:
                    break;
                }
              }));

      Pose3d reefBranch =
          FieldConstants.Reef.branchPositions.get(4).get(FieldConstants.ReefHeight.L1);
      controlScheme
          .driveToReef()
          .whileTrue(
              Commands.run(
                  () ->
                      DriveCommands.driveToPointMA(
                          reefBranch
                              .toPose2d()
                              .transformBy(
                                  new Transform2d(
                                      Inches.of(2.25).plus(Constants.robotScoringOffset),
                                      Inches.of(1.8),
                                      Rotation2d.k180deg)),
                          drivetrain),
                  drivetrain));
    }
    */
    // RESET ENCODER
    //    controlScheme
    //        .getController()
    //        .b()
    //        .onTrue(
    //            arm.resetEncoder()
    //                .andThen(
    //                    Commands.waitUntil(
    //                        () -> arm.getPosition() ==
    // Rotations.of(Degrees.of(-90).in(Rotations))))
    //                .andThen(arm.park()));

    // CLIMBER
    //    controlScheme.getController().leftTrigger().whileTrue(climber.climberOut(Volts.of(-12)));
    //    controlScheme.getController().rightTrigger().whileTrue(climber.climberOut(Volts.of(8)));

    // CLAW
    //    primaryController.a().whileTrue(claw.extake());

    // CHANGE SUPER STRUCTURE LEVEL
    primaryController.povRight().onTrue(scoreCommands.intakeCoral()); // D-PAD RIGHT
    primaryController.povDown().onTrue(scoreCommands.bottomLevel()); // D-PAD DOWN
    primaryController.povLeft().onTrue(scoreCommands.midLevel()); // D-PAD LEFT

    //    var topLevel = scoreCommands.topLevel();
    //    SmartDashboard.putData("topLevel", topLevel);
    primaryController.povUp().onTrue(scoreCommands.topLevel()); // D-PAD UP

    //    var score = scoreCommands.score();
    //    SmartDashboard.putData("scoreCommand", score);

    // MOVE ARM
    //    primaryController.a().onTrue(arm.intake());
    primaryController.a().onTrue(scoreCommands.score());
    primaryController.y().onTrue(arm.L2());
    primaryController.x().onTrue(arm.L1());

    //    primaryController
    //        .b()
    //        .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
    // CommandScheduler.getInstance().printWatchdogEpochs();

    // TODO: MAKE EVERYTHING INTERRUPTABLE

    // DRIVE TO STATION
    primaryController
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () ->
                    DriveCommands.driveToPointMA(
                        FieldConstants.CoralStation.rightCenterFace.transformBy(
                            FieldConstants.CoralStation.coralOffset),
                        drivetrain,
                        true),
                drivetrain));

    primaryController
        .leftTrigger()
        .whileTrue(
            Commands.run(
                () ->
                    DriveCommands.driveToPointMA(
                        FieldConstants.CoralStation.leftCenterFace.transformBy(
                            FieldConstants.CoralStation.coralOffset),
                        drivetrain,
                        true),
                drivetrain));

    // DRIVE TO REEF

    primaryController
        .rightBumper()
        .onTrue(Commands.runOnce(() -> chooseReefBranch(aprilTagToBranch.aprilTagToBranch(true))))
        .whileTrue(
            Commands.run(
                () ->
                    DriveCommands.driveToPointMA(
                        reefBranch.transformBy(FieldConstants.Reef.reefOffset), drivetrain),
                drivetrain));
    primaryController
        .leftBumper()
        .onTrue(Commands.runOnce(() -> chooseReefBranch(aprilTagToBranch.aprilTagToBranch(false))))
        .whileTrue(
            Commands.run(
                () ->
                    DriveCommands.driveToPointMA(
                        reefBranch.transformBy(FieldConstants.Reef.leftReefOffset), drivetrain),
                drivetrain));

    primaryController.rightStick().onTrue(Commands.runOnce(arm::reconfigPID));

    primaryController
            .start()
            .onTrue(Commands.runOnce(() -> chooseReefBranch(aprilTagToBranch.aprilTagToBranch(true))))
            .whileTrue(
                    Commands.run(
                            () ->
                                    DriveCommands.driveToPointMA(
                                            reefBranch.transformBy(FieldConstants.Reef.reefOffsetForward), drivetrain),
                            drivetrain));

    primaryController
            .back()
            .onTrue(Commands.runOnce(() -> chooseReefBranch(aprilTagToBranch.aprilTagToBranch(false))))
            .whileTrue(
                    Commands.run(
                            () ->
                                    DriveCommands.driveToPointMA(
                                            reefBranch.transformBy(FieldConstants.Reef.leftReefOffsetForward), drivetrain),
                            drivetrain));

    secondController.povRight().onTrue(scoreCommands.intakeCoral()); // D-PAD RIGHT
    secondController.povDown().onTrue(scoreCommands.bottomLevel()); // D-PAD DOWN
    secondController.povLeft().onTrue(scoreCommands.midLevel()); // D-PAD LEFT
    secondController.povUp().onTrue(scoreCommands.topLevel()); // D-PAD UP

    secondController.b().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

    secondController.rightTrigger().whileTrue(climber.climberOut(Volts.of(12)));
    secondController.leftTrigger().whileTrue(climber.climberOut(Volts.of(-12)));

    secondController.rightBumper().whileTrue(claw.intake());
    secondController.leftBumper().whileTrue(claw.extake());

    //    secondController.leftTrigger().whileTrue(counterWeight.counterWeightIn());
    //    secondController.rightTrigger().whileTrue(counterWeight.counterWeightOut());

    // controlScheme.getController().start().onTrue(Commands.runOnce(DriveCommands::reconfigurePID));

  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void chooseReefBranch(int branchNumber) {
    if (branchNumber < 0) {
      return;
    }
    reefBranch =
        FieldConstants.Reef.branchPositions
            .get(branchNumber)
            .get(FieldConstants.ReefHeight.L4)
            .toPose2d();
    SmartDashboard.putNumber("reefBranch", branchNumber);
  }
}
