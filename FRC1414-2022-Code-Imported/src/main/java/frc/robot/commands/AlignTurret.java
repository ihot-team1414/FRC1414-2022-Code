package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimbSubsystem.PivotPosition;
import frc.robot.subsystems.ClimbSubsystem;

public class AlignTurret extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final ClimbSubsystem climbSubsystem;

  public AlignTurret(TurretSubsystem turretSubsystem, ClimbSubsystem climbSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.climbSubsystem = climbSubsystem;

    addRequirements(turretSubsystem, climbSubsystem);
  }

  @Override
  public void execute() {
    // If the pivot is not at the vertical position, move it to the vertical position, then run targetting.
    if (!climbSubsystem.isPivotAtTarget(PivotPosition.Vertical)) {
      turretSubsystem.home();
      climbSubsystem.setPivot(PivotPosition.Vertical);
    } else {
      turretSubsystem.visionTargeting();
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.home();
  }
}
