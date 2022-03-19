package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerAutoCommand extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final BooleanSupplier shooting;

    public IndexerAutoCommand(IndexerSubsystem indexerSubsystem, BooleanSupplier shooting) {
        this.indexerSubsystem = indexerSubsystem;
        this.shooting = shooting;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.shoot();
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopLoader();
    }
}
