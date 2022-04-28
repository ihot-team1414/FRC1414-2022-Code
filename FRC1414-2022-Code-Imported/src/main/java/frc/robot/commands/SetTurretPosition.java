package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimbSubsystem.PivotPosition;
import frc.robot.subsystems.ClimbSubsystem.TelescopePosition;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class SetTurretPosition extends CommandBase {
  private final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
  private final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
  private double pos;

  public SetTurretPosition(double pos) {
    this.pos = pos;
    addRequirements(turretSubsystem, climbSubsystem);
  }

  @Override
  public void execute() {
    if (!climbSubsystem.isPivotAtTarget(PivotPosition.Vertical)) {
      climbSubsystem.setPivot(PivotPosition.Vertical);
      turretSubsystem.home();
    } else {
      turretSubsystem.setPosition(pos);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(pos - turretSubsystem.getPosition()) < Constants.TURRET_POSITION_ALLOWED_ERROR;
  }
}
