package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveAutoCommand2 extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final Pose2d m_targetPos;

    public DriveAutoCommand2(DrivetrainSubsystem drivetrainSubsystem, Pose2d targetPos) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_targetPos = targetPos;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        Pose2d targetPos = m_targetPos;

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 1));
        
    }

    @Override
    public void end(boolean interrupted) {
    }
}
