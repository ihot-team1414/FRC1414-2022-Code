package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class AlignHood extends CommandBase {
  private final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();

  public AlignHood() {
    addRequirements(hoodSubsystem);
  }

  @Override
  public void execute() {
    hoodSubsystem.visionTargeting();
  }

  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.home();
  }
}
