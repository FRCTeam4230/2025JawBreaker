package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimulationControlScheme extends DefaultControlScheme {

  /* LoggedDashboardChooser<IntakeSide> intakeSideChooser =
      new LoggedDashboardChooser<IntakeSide>("Intake Side");
  LoggedDashboardChooser<CoralBranch> coralBranchChooser =
      new LoggedDashboardChooser<CoralBranch>("Coral Branch");*/

  public SimulationControlScheme() {
    super();
    /* intakeSideChooser.addDefaultOption("Left", IntakeSide.LEFT);
    intakeSideChooser.addOption("Right", IntakeSide.RIGHT);

    coralBranchChooser.addDefaultOption("A", CoralBranch.A);
    coralBranchChooser.addOption("B", CoralBranch.B);
    coralBranchChooser.addOption("C", CoralBranch.C);
    coralBranchChooser.addOption("D", CoralBranch.D);
    coralBranchChooser.addOption("E", CoralBranch.E);
    coralBranchChooser.addOption("F", CoralBranch.F);
    coralBranchChooser.addOption("G", CoralBranch.G);
    coralBranchChooser.addOption("H", CoralBranch.H);
    coralBranchChooser.addOption("I", CoralBranch.I);
    coralBranchChooser.addOption("J", CoralBranch.J);
    coralBranchChooser.addOption("K", CoralBranch.K);
    coralBranchChooser.addOption("L", CoralBranch.L);*/
  }

  @Override
  public Trigger setLoadGamePiece() {
    return controller.rightBumper();
  }

  @Override
  public Trigger setUnloadGamePiece() {
    return controller.leftBumper();
  }
}
