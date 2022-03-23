package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TurretSubsystem;

public class TurretEjectCommand extends CommandBase {
    private final TurretSubsystem turretSubsystem;

    public TurretEjectCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        turretSubsystem.eject();
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.home();
    }
}
