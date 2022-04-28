package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class Spool extends CommandBase {
  private final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();

  public Spool() {
    addRequirements(climbSubsystem);
  }

  @Override
  public void execute() {
    climbSubsystem.spool();
  }
}
