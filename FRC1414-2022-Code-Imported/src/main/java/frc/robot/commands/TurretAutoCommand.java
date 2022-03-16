package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TurretSubsystem;

public class TurretAutoCommand extends CommandBase {
    private final TurretSubsystem turretSubsystem;

    public TurretAutoCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        turretSubsystem.visionTargeting();
    }
}
