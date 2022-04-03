package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimbSubsystem.PivotPosition;
import frc.robot.subsystems.ClimbSubsystem.TelescopePosition;
import frc.robot.subsystems.ClimbSubsystem;

public class AlignTurret extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final ClimbSubsystem climbSubsystem;

  public AlignTurret(TurretSubsystem turretSubsystem, ClimbSubsystem climbSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.climbSubsystem = climbSubsystem;

    addRequirements(turretSubsystem, climbSubsystem);

    SmartDashboard.putBoolean("ALIGN", false);
  }

  @Override
  public void execute() {
    if (!climbSubsystem.isPivotAtTarget(PivotPosition.Vertical)) {
      climbSubsystem.setPivot(PivotPosition.Vertical);
      // climbSubsystem.setTelescope(TelescopePosition.Starting);
      turretSubsystem.home();
    } else {
      turretSubsystem.visionTargeting();
    }

    SmartDashboard.putBoolean("ALIGN", true);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.home();
    SmartDashboard.putBoolean("ALIGN", false);
  }
}
