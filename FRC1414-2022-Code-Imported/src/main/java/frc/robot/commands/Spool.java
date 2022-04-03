package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class Spool extends CommandBase {
  private final ClimbSubsystem climbSubsystem;

  public Spool(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  @Override
  public void execute() {
    climbSubsystem.spool();
  }
}
