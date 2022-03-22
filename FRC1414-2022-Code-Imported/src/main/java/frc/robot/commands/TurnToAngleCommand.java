package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnToAngleCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final double angle;

    public TurnToAngleCommand(DrivetrainSubsystem drivetrainSubsystem, double angle) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.angle = angle;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, m_drivetrainSubsystem.getRequiredTurningSpeedForAngle(angle)));
        
    }

    @Override
    public void end(boolean interrupted) {

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));

    }
}
