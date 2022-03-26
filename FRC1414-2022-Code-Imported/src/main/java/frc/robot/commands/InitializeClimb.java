package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.PivotPosition;

public class InitializeClimb extends CommandBase {
  private ClimbSubsystem climbSubsystem;

  public InitializeClimb(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.setPivot(PivotPosition.Vertical);
  }
}
