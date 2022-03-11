package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TurretCommand extends CommandBase {
    private final TurretSubsystem m_turretSubsystem;
    private final DoubleSupplier m_manualControlSupplier;

    public TurretCommand(TurretSubsystem turretSubsystem, DoubleSupplier manualControlSupplier) {
        this.m_turretSubsystem = turretSubsystem;
        this.m_manualControlSupplier = manualControlSupplier;
        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        m_turretSubsystem.visionTargeting();
        m_turretSubsystem.moveTurret(m_manualControlSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
    }
}
