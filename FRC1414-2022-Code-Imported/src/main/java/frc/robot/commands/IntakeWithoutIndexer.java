package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeWithoutIndexer extends CommandBase {
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

  public IntakeWithoutIndexer() {
    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.open();
    intakeSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    intakeSubsystem.close();
  }
}