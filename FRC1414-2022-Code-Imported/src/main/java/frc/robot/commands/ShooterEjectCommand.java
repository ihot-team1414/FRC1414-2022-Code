package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ShooterSubsystem;

public class ShooterEjectCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;

    public ShooterEjectCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.eject();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }
}
