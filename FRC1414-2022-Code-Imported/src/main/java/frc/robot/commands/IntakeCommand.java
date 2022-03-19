package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class IntakeCommand extends CommandBase {

  private final IntakeSubsystem intake;
  private final DoubleSupplier throttle;
  private final BooleanSupplier reverse;

  public IntakeCommand(IntakeSubsystem intake, DoubleSupplier throttle, BooleanSupplier reverse) {
    this.intake = intake;
    this.throttle = throttle;
    this.reverse = reverse;
    addRequirements(this.intake);
  }

  public void initialize() {
  }

  public void execute() {
    if ((this.throttle.getAsDouble() > 0.75)) {
      if (this.reverse.getAsBoolean()) {
        this.intake.setIntakeSpeed(-Constants.INTAKE_SPEED);
      } else {
        this.intake.setIntakeSpeed(Constants.INTAKE_SPEED);
      }
    } else {
        this.intake.setIntakeSpeed(0);
    }
  }

  @Override
  public boolean isFinished() {
      return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.intake.setIntakeSpeed(0);
}
}