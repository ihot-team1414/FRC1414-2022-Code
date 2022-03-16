package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ArmPosition;

public class RobotStartCommand extends CommandBase {
    private final ClimbSubsystem climbSubsystem;

    public RobotStartCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setArmPosition(ArmPosition.Vertical);
    }
}
