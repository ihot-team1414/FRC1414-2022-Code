package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ArmPosition;

public class ArmStartCommand extends CommandBase {
    private ClimbSubsystem climbSubsystem;
    
    public ArmStartCommand(ClimbSubsystem climbSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climbSubsystem);

        this.climbSubsystem = climbSubsystem;
    }

    @Override
    public void execute() {
        climbSubsystem.setArmPosition(ArmPosition.Vertical);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
