package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DescheduleSubsystem extends CommandBase {

  public DescheduleSubsystem(Subsystem subsystem) {

    addRequirements(subsystem);
  }
}
