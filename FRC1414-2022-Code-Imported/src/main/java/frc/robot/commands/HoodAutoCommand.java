package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.HoodSubsystem;

public class HoodAutoCommand extends CommandBase {
    private final HoodSubsystem hoodSubsystem;

    public HoodAutoCommand(HoodSubsystem hoodSubsystem) {
        this.hoodSubsystem = hoodSubsystem;

        addRequirements(hoodSubsystem);
    }

    @Override
    public void execute() {
        hoodSubsystem.visionTargeting();
    }
}
