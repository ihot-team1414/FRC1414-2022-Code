package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAutoDeployCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    
    public IntakeAutoDeployCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        this.intakeSubsystem.setForward();
        this.intakeSubsystem.setIntakeSpeed(speed);
    }
}
