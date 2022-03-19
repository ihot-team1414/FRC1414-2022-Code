package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAutoDeployCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    
    public IntakeAutoDeployCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.intakeSubsystem.setForward();
        this.intakeSubsystem.setIntakeSpeed(-Constants.INTAKE_SPEED);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.stop();
    }
}
