package frc.robot.autos;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.List;

public class FiveBallAuto implements AutoInterface {
    private ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.DRIVETRAIN_PATH_THETA_kP, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);

    private Trajectory[] trajectories = {
            TrajectoryGenerator.generateTrajectory(
                    Constants.STARTING_POSITIONS[0],
                    List.of(),
                    new Pose2d(7.9, 1, Rotation2d.fromDegrees(-90)),
                    Constants.TRAJECTORY_CONFIG),
            TrajectoryGenerator.generateTrajectory(
                    new Pose2d(7.9, 1, Rotation2d.fromDegrees(-90)),
                    List.of(new Translation2d(6, 1.02)),
                    new Pose2d(5, 1.5, Rotation2d.fromDegrees(85)),
                    Constants.TRAJECTORY_CONFIG),
            TrajectoryGenerator.generateTrajectory(
                    new Pose2d(5, 1.5, Rotation2d.fromDegrees(85)),
                    List.of(),
                    new Pose2d(1.3, 1.2, Rotation2d.fromDegrees(225)),
                    Constants.TRAJECTORY_CONFIG),
            TrajectoryGenerator.generateTrajectory(
                    new Pose2d(1.3, 1.2, Rotation2d.fromDegrees(225)),
                    List.of(),
                    new Pose2d(3, 3, Rotation2d.fromDegrees(0)),
                    Constants.TRAJECTORY_CONFIG)
    };

    private SequentialCommandGroup auto;

    public FiveBallAuto(
            DrivetrainSubsystem drivetrainSubsystem,
            IntakeSubsystem intakeSubsystem,
            IndexerSubsystem indexerSubsystem,
            ShooterSubsystem shooterSubsystem,
            TurretSubsystem turretSubsystem,
            HoodSubsystem hoodSubsystem) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        auto = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new FollowTrajectory(
                                        drivetrainSubsystem,
                                        trajectories[0]),
                                new FollowTrajectory(
                                        drivetrainSubsystem,
                                        trajectories[1])))
                        .deadlineWith(new Intake(indexerSubsystem, intakeSubsystem)),
                new ParallelCommandGroup(
                        new AutoShoot(shooterSubsystem, indexerSubsystem, hoodSubsystem).withTimeout(4))
                        .deadlineWith(new IntakeRoll(intakeSubsystem)),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new FollowTrajectory(
                                        drivetrainSubsystem,
                                        trajectories[2]),
                                new WaitCommand(0.5)))
                        .deadlineWith(new Intake(indexerSubsystem, intakeSubsystem).withTimeout(5)),
                new FollowTrajectory(
                        drivetrainSubsystem,
                        trajectories[3]),
                new ParallelCommandGroup(new AutoShoot(shooterSubsystem, indexerSubsystem, hoodSubsystem),
                        new IntakeRoll(intakeSubsystem)));
    }

    public SequentialCommandGroup getAuto() {
        return auto;
    }
}
