package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexerAutoCommand extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;

    public IndexerAutoCommand(IndexerSubsystem indexerSubsystem, ShooterSubsystem shooter) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.load();
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stop();
    }
}
