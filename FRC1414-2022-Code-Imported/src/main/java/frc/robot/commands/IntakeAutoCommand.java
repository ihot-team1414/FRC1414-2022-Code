package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAutoCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeAutoCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeSpeed(Constants.INTAKE_SPEED);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setForward();
    }

}
