package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ArmPosition;
import frc.robot.subsystems.ClimbSubsystem.ElevatorPosition;

public class MoveClimbCommand extends CommandBase {
    private final ClimbSubsystem climbSubsystem;
    private final ArmPosition armPosition;
    private final ElevatorPosition elevatorPosition;

    public MoveClimbCommand(ClimbSubsystem climbSubsystem, ArmPosition armPosition, ElevatorPosition elevatorPosition) {
        this.climbSubsystem = climbSubsystem;
        this.armPosition = armPosition;
        this.elevatorPosition = elevatorPosition;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setArmPosition(armPosition);
        climbSubsystem.setElevatorPosition(elevatorPosition);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isArmAtTarget(armPosition) && climbSubsystem.isElevatorAtTarget(elevatorPosition);
    }
}
