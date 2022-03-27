package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class AlignHood extends CommandBase {
  private final HoodSubsystem hoodSubsystem;

  public AlignHood(HoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;

    addRequirements(hoodSubsystem);
  }

  @Override
  public void execute() {
    // hoodSubsystem.visionTargeting();
  }

  @Override
  public void end(boolean interrupted) {
    // hoodSubsystem.home();
  }
}
