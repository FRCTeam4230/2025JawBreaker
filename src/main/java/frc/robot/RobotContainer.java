package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.controls.ControlScheme;
import frc.robot.controls.DefaultControlScheme;
import frc.robot.controls.SimulationControlScheme;
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
import frc.robot.subsystems.counterweight.CounterWeight;
import frc.robot.subsystems.counterweight.CounterWeightIOREV;
import frc.robot.subsystems.counterweight.CounterWeightIOSIM;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingAngle;
import frc.robot.subsystems.drive.requests.SwerveSetpointGen;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOREV;
import frc.robot.subsystems.elevator.ElevatorIOSIMREV;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

public class RobotContainer {

  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final ControlScheme controlScheme;

  private final TunableController testJoystick =
      new TunableController(1).withControllerType(TunableControllerType.QUADRATIC);

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
  private final CounterWeight counterWeight;

  private final ScoringCommands scoreCommands;

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
            new VisionIOLimelight("limelight-fl", drivetrain::getVisionParameters),
            new VisionIOLimelight("limelight-back", drivetrain::getVisionParameters));

        elevator = new Elevator(new ElevatorIOREV() {});
        arm = new Arm(new ArmIOREV() {});
        claw = new Claw(new ClawIOREV() {});
        climber = new Climber(new ClimberIOREV() {});
        counterWeight = new CounterWeight(new CounterWeightIOREV());

        controlScheme = new DefaultControlScheme();

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new Drive(currentDriveTrain);

        new Vision(
            drivetrain::addVisionData,
            new VisionIOPhotonVisionSIM(
                "Front Camera",
                new Transform3d(
                    new Translation3d(0.2, 0.0, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(0))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Back Camera",
                new Transform3d(
                    new Translation3d(-0.2, 0.0, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(180))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Left Camera",
                new Transform3d(
                    new Translation3d(0.0, 0.2, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(90))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Right Camera",
                new Transform3d(
                    new Translation3d(0.0, -0.2, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(-90))),
                drivetrain::getVisionParameters));

        elevator = new Elevator(new ElevatorIOSIMREV());
        arm = new Arm(new ArmIOREVSIM());
        claw = new Claw(new ClawIOSIMREV());
        climber = new Climber(new ClimberIOSIM());
        counterWeight = new CounterWeight(new CounterWeightIOSIM());
        controlScheme = new SimulationControlScheme();

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
        counterWeight = new CounterWeight(new CounterWeightIOREV());
        controlScheme = new DefaultControlScheme();
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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
    scoreCommands = new ScoringCommands(elevator, arm, claw);

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
                            controlScheme
                                .getFieldX()
                                .get())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            controlScheme.getFieldY().get())) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            controlScheme
                                .getFieldRotation()
                                .get())))); // Drive counterclockwise with negative X (left)

    controlScheme
        .getController()
        .back()
        .onTrue(Commands.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));
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
    SwerveSetpointGen setpointGen =
        new SwerveSetpointGen(
                drivetrain.getChassisSpeeds(),
                drivetrain.getModuleStates(),
                drivetrain::getRotation)
            .withDeadband(MaxSpeed.times(0.05))
            .withRotationalDeadband(MaxAngularRate.times(0.05))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
    ProfiledFieldCentricFacingAngle driveFacingAngle =
        new ProfiledFieldCentricFacingAngle(
                new TrapezoidProfile.Constraints(
                    MaxAngularRate.baseUnitMagnitude(),
                    MaxAngularRate.div(0.25).baseUnitMagnitude()))
            .withDeadband(MaxSpeed.times(0.05))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Set PID for ProfiledFieldCentricFacingAngle
    driveFacingAngle.HeadingController.setPID(7, 0, 0);

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
    /*
        joystick.leftTrigger().whileTrue(climber.climberOut(Volts.of(-12)));
        joystick.rightTrigger().whileTrue(climber.climberOut(Volts.of(8)));

        joystick.rightBumper().whileTrue(claw.intake());
        joystick.leftBumper().whileTrue(claw.extake());

        joystick.a().onTrue(arm.intake().andThen(elevator.intake()));
        joystick.x().onTrue(arm.L1());
        joystick.y().onTrue(arm.L2());
        joystick.b().onTrue(scoreCommands.stopAll().andThen(counterWeight.counterWeightStop()));

        joystick.povDown().onTrue(scoreCommands.intakeCoral());

        joystick.povLeft().onTrue(scoreCommands.midLevel());
        joystick.povUp().onTrue(scoreCommands.topLevel());

        joystick
            .povRight()
            .onTrue(arm.intake().andThen(Commands.waitSeconds(0.25)).andThen(elevator.L2()));
    */
    // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // testJoystick.back().onTrue(arm.reconfigPID());

    /**** COUNTER WEIGHT TEST ********/
    //    joystick
    //        .rightBumper()
    //        .whileTrue(counterWeight.counterWeightOut()); // TODO: CONSTANTS and change this.
    //    joystick.leftBumper().whileTrue(counterWeight.counterWeightIn());

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
    controlScheme
        .getController()
        .b()
        .onTrue(
            arm.resetEncoder()
                .andThen(
                    Commands.waitUntil(
                        () -> arm.getPosition() == Rotations.of(Degrees.of(-90).in(Rotations))))
                .andThen(arm.park()));

    // CLIMBER
    controlScheme.getController().leftTrigger().whileTrue(climber.climberOut(Volts.of(-12)));
    controlScheme.getController().rightTrigger().whileTrue(climber.climberOut(Volts.of(8)));

    // CLAW
    controlScheme.getController().rightBumper().whileTrue(claw.intake());
    controlScheme.getController().leftBumper().whileTrue(claw.extake());

    //    controlScheme
    //        .getController()
    //        .b()
    //        .onTrue(scoreCommands.stopAll().andThen(counterWeight.counterWeightStop()));

    controlScheme.getIntake().onTrue(scoreCommands.intakeCoral()); // D-PAD RIGHT
    controlScheme.getL1().onTrue(scoreCommands.bottomLevel()); // D-PAD DOWN
    controlScheme.getL2().onTrue(scoreCommands.midLevel()); // D-PAD LEFT
    controlScheme.getL4().onTrue(scoreCommands.topLevel()); // D-PAD UP

    controlScheme.getController().x().onTrue(arm.L1());
    controlScheme.getController().b().onTrue(scoreCommands.stopAll());

    //    controlScheme.getController()
    //            .povRight()
    //            .onTrue(arm.intake().andThen(Commands.waitSeconds(0.25)).andThen(elevator.L2()));

    //    controlScheme
    //        .getController()
    //        .y()
    //        .whileTrue(new ReefAlignCommand(drivetrain, ReefAlignCommand.StationType.REEF,
    // false));
    //    controlScheme
    //        .getController()
    //        .a()
    //        .whileTrue(new ReefAlignCommand(drivetrain, ReefAlignCommand.StationType.REFILL,
    // false));
    controlScheme
        .getController()
        .a()
        .whileTrue(
            Commands.run(
                () ->
                    DriveCommands.driveToPointMA(
                        FieldConstants.CoralStation.leftCenterFace.transformBy(
                            FieldConstants.CoralStation.coralOffset),
                        drivetrain,
                        true),
                drivetrain));
    Pose3d reefBranch =
        FieldConstants.Reef.branchPositions.get(2).get(FieldConstants.ReefHeight.L4);
    controlScheme
        .getController()
        .y()
        .whileTrue(
            Commands.run(
                () ->
                    DriveCommands.driveToPointMA(
                        reefBranch.toPose2d().transformBy(FieldConstants.Reef.reefOffset),
                        drivetrain),
                drivetrain));

    controlScheme.getController().start().onTrue(Commands.runOnce(DriveCommands::reconfigurePID));
  }

  public void setupNamedCommands() {

    NamedCommands.registerCommand("Prep L1", arm.L1());

    //    NamedCommands.registerCommand(
    //        "Prepare L4",
    //        Commands.runOnce(() -> SmartController.targetReefHeight =
    // FieldConstants.ReefHeight.L4));
    //    NamedCommands.registerCommand(
    //        "Prepare L3",
    //        Commands.runOnce(() -> SmartController.targetReefHeight =
    // FieldConstants.ReefHeight.L3));
    //    NamedCommands.registerCommand(
    //        "Prepare L2",
    //        Commands.runOnce(() -> SmartController.targetReefHeight =
    // FieldConstants.ReefHeight.L2));
    //    NamedCommands.registerCommand(
    //        "Prepare L1",
    //        Commands.runOnce(() -> SmartController.targetReefHeight =
    // FieldConstants.ReefHeight.L1));
    //    NamedCommands.registerCommand(
    //        "Shoot",
    //        Commands.waitUntil(superstructure.isAtTarget())
    //            .andThen(new WaitCommand(0.5))
    //            .andThen(coralWrist.gamePieceUnloaded()));
    NamedCommands.registerCommand("Intake", scoreCommands.intakeCoral());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
