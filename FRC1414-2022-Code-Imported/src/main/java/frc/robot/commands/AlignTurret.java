package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimbSubsystem.PivotPosition;
import frc.robot.subsystems.ClimbSubsystem.TelescopePosition;
import frc.robot.subsystems.ClimbSubsystem;

public class AlignTurret extends CommandBase {
  private final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
  private final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();

  public AlignTurret() {
    addRequirements(turretSubsystem, climbSubsystem);
  }

  @Override
  public void execute() {
    if (!climbSubsystem.isPivotAtTarget(PivotPosition.Vertical) && !climbSubsystem.isTelescopeAtTarget(TelescopePosition.Starting)) {
      climbSubsystem.setPivot(PivotPosition.Vertical);
      climbSubsystem.setTelescope(TelescopePosition.Starting);
      turretSubsystem.home();
    } else {
      turretSubsystem.visionTargeting();
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.home();
  }
}
