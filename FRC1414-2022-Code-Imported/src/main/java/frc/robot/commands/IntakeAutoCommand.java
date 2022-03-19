package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAutoCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final BooleanSupplier negative;

    public IntakeAutoCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier negative) {
        this.intakeSubsystem = intakeSubsystem;
        this.negative = negative;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        var speed = Constants.INTAKE_SPEED;
        if(negative.getAsBoolean()) {
            speed *= -1;
        }
        intakeSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setForward();
    }

}
