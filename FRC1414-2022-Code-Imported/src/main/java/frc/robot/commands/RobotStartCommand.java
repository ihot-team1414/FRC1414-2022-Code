package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.PivotPosition;
import frc.robot.subsystems.ClimbSubsystem.TelescopePosition;

public class RobotStartCommand extends CommandBase {
    private final ClimbSubsystem climbSubsystem;

    public RobotStartCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setPivot(PivotPosition.Vertical);
        climbSubsystem.setTelescope(TelescopePosition.Neutral);
    }
}
