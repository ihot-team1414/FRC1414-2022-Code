package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.PivotPosition;
import frc.robot.subsystems.ClimbSubsystem.TelescopePosition;

public class MoveClimb extends CommandBase {
  private final ClimbSubsystem climbSubsystem;
  private final PivotPosition armPosition;
  private final TelescopePosition elevatorPosition;

  public MoveClimb(ClimbSubsystem climbSubsystem, PivotPosition armPosition,
      TelescopePosition elevatorPosition) {
    this.climbSubsystem = climbSubsystem;
    this.armPosition = armPosition;
    this.elevatorPosition = elevatorPosition;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.setPivot(armPosition);
    climbSubsystem.setTelescope(elevatorPosition);
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.isPivotAtTarget(armPosition) && climbSubsystem.isTelescopeAtTarget(elevatorPosition);
  }

  @Override
  public void end(boolean interrupted) {

  }
}
