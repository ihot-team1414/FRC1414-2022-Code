package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveAutoCommand2 extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;


    public DriveAutoCommand2(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 2));
        
    }

    @Override
    public void end(boolean interrupted) {

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));

    }
}
