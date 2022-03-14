package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HoodCommand extends CommandBase {
    private final HoodSubsystem m_hoodSubsystem;


    public HoodCommand(HoodSubsystem hoodSubsystem) {
        this.m_hoodSubsystem = hoodSubsystem;

        addRequirements(hoodSubsystem);
    }

    @Override
    public void execute() {

        // m_hoodSubsystem.visionTargeting();
        
    }

    @Override
    public void end(boolean interrupted) {
    }
}
