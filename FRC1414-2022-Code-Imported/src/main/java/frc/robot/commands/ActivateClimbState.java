package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ActivateClimbState extends CommandBase {
  private final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
  private final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();

  public ActivateClimbState() {
    addRequirements(climbSubsystem, turretSubsystem);
  }

  @Override
  public void execute() {
    if (!turretSubsystem.isHome()) {
      turretSubsystem.home();
    } else {
      climbSubsystem.activateState();
    }
  }
}
