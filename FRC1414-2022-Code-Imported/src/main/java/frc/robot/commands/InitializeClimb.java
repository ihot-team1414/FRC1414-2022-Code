package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.PivotPosition;
import frc.robot.subsystems.ClimbSubsystem.TelescopePosition;

public class InitializeClimb extends CommandBase {
  private ClimbSubsystem climbSubsystem;

  public InitializeClimb(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.resetState();
    climbSubsystem.setTelescope(TelescopePosition.Neutral);
    climbSubsystem.setPivot(PivotPosition.Vertical);
  }
}
