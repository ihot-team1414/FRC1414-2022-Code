package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAutoCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;

    public ShooterAutoCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.shoot();
    }
}
