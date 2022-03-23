package frc.robot.commands;

import java.util.function.BooleanSupplier;

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
        var speed = Constants.INTAKE_SPEED;
        intakeSubsystem.set(speed);
    }

    @Override
    public void initialize() {
        intakeSubsystem.open();
    }

}
