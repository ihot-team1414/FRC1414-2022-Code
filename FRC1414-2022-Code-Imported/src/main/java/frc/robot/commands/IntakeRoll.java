package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRoll extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;

  public IntakeRoll(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.open();
    intakeSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.stop();
    this.intakeSubsystem.close();
  }
}