// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeIndexerCommand extends CommandBase {
  private final IndexerSubsystem m_indexer;
  private final IntakeSubsystem m_intake;
  private final BooleanSupplier m_intakingSupplier;
  private final BooleanSupplier m_outtakingSupplier;
  private final BooleanSupplier m_loadingSupplier;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeIndexerCommand(IntakeSubsystem intake, IndexerSubsystem indexer, BooleanSupplier intakingSupplier, BooleanSupplier outtakingSupplier, BooleanSupplier loadingSupplier) {
    m_intake = intake;
    m_indexer = indexer;
    m_outtakingSupplier = outtakingSupplier;
    m_intakingSupplier = intakingSupplier;
    m_loadingSupplier = loadingSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean intaking = m_intakingSupplier.getAsBoolean();
    boolean outtaking = m_outtakingSupplier.getAsBoolean();
    boolean loading = m_loadingSupplier.getAsBoolean();

    if(intaking) {
        m_intake.intake();
        m_indexer.holdBall();
    } else if (loading) {
        m_indexer.shoot();
    } else if(outtaking) {
        m_intake.outtake();
        m_indexer.eject();
    } else {
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
