package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.FourBallAuto;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.util.Utils;

public class RobotContainer {
  // CONTROLLERS
  private final XboxController driver = new XboxController(1);
  private final XboxController operator = new XboxController(0);

  // SUBSYSTEMS
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(Constants.STARTING_POSITIONS[0]);

  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();

  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  private final TurretSubsystem turretSubsystem = new TurretSubsystem();

  // AUTOS
  private SendableChooser<Command> chooser = new SendableChooser<>();

  private final FourBallAuto fourBallAuto = new FourBallAuto(
      drivetrainSubsystem,
      intakeSubsystem,
      indexerSubsystem,
      shooterSubsystem,
      turretSubsystem,
      hoodSubsystem);

  // private final TwoBallAuto twoBallAuto = new TwoBallAuto(
  // drivetrainSubsystem,
  // intakeSubsystem,
  // indexerSubsystem,
  // shooterSubsystem,
  // turretSubsystem,
  // hoodSubsystem);

  TrajectoryConfig config;

  public RobotContainer() {
    // AUTO CHOOSER
    
    ArrayList<Translation2d> list = new ArrayList<>();
    list.add(new Translation2d(6, 4.75));
    list.add(new Translation2d(7, 5.25));

    SmartDashboard.putData("Auto Chooser", this.chooser);

    // chooser.addOption("Wait", new WaitCommand(15));
    chooser.addOption("5 Ball", new SequentialCommandGroup(
          new Intake(indexerSubsystem, intakeSubsystem).alongWith(new ParallelCommandGroup(
            new SequentialCommandGroup(
                new FollowTrajectory(
                    drivetrainSubsystem,
                    TrajectoryGenerator.generateTrajectory(
                        Constants.STARTING_POSITIONS[0],
                        List.of(),
                        new Pose2d(7.9, 1.1, Rotation2d.fromDegrees(-90)),
                        Constants.TRAJECTORY_CONFIG)),
                new FollowTrajectory(
                    drivetrainSubsystem,
                    TrajectoryGenerator.generateTrajectory(
                        new Pose2d(7.9, 1.1, Rotation2d.fromDegrees(-90)),
                        List.of(new Translation2d(6.5, 1.25)),
                        new Pose2d(4.7, 1.45, Rotation2d.fromDegrees(90)),
                        Constants.TRAJECTORY_CONFIG)))
        ),
        new Shoot(shooterSubsystem, indexerSubsystem).withTimeout(2),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new FollowTrajectory(
                    drivetrainSubsystem,
                    TrajectoryGenerator.generateTrajectory(
                        new Pose2d(4.7, 1.45, Rotation2d.fromDegrees(90)),
                        List.of(),
                        new Pose2d(0.8, 1, Rotation2d.fromDegrees(225)),
                        Constants.TRAJECTORY_CONFIG)),
                new WaitCommand(2)),
            new Intake(indexerSubsystem, intakeSubsystem).withTimeout(5)),
        new FollowTrajectory(
            drivetrainSubsystem,
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0.8, 1, Rotation2d.fromDegrees(225)),
                List.of(),
                new Pose2d(2.5, 2.5, Rotation2d.fromDegrees(0)),
                Constants.TRAJECTORY_CONFIG)),
        new Shoot(shooterSubsystem, indexerSubsystem))));

    // chooser.addOption("4 Ball Outside", fourBallAuto.getAuto());
    // chooser.addOption("2 Ball High", twoBallAuto.getAuto());

    // DEFAULT COMMANDS
    // hoodSubsystem.setDefaultCommand(new AlignHood(hoodSubsystem));

    // The align turret command checks to see if the pivot arms are in the vertical
    // position, otherwise, it homes.
    turretSubsystem.setDefaultCommand(new AlignTurret(turretSubsystem, climbSubsystem));

    drivetrainSubsystem.setDefaultCommand(new Drive(
        drivetrainSubsystem,
        () -> Utils.deadband(driver.getRightY(), 0.1),
        () -> Utils.deadband(driver.getRightX(), 0.1),
        () -> Utils.deadband(driver.getLeftX(), 0.1),
        () -> driver.getRightBumper(),
        () -> driver.getLeftBumper()));

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // DRIVER CONTROLS

    // Start button to zero gyroscope
    new JoystickButton(driver, Button.kStart.value).whenPressed(() -> drivetrainSubsystem.zeroGyroscope());

    // Driver buttons for turn to angle
    new JoystickButton(driver, Button.kA.value).whileActiveContinuous(new TurnToAngle(drivetrainSubsystem,
        () -> Utils.deadband(driver.getRightX(), 0.1), () -> Utils.deadband(driver.getRightY(), 0.1), 180));
    new JoystickButton(driver, Button.kX.value).whileActiveContinuous(new TurnToAngle(drivetrainSubsystem,
        () -> Utils.deadband(driver.getRightX(), 0.1), () -> Utils.deadband(driver.getRightY(), 0.1), 90));
    new JoystickButton(driver, Button.kB.value).whileActiveContinuous(new TurnToAngle(drivetrainSubsystem,
        () -> Utils.deadband(driver.getRightX(), 0.1), () -> Utils.deadband(driver.getRightY(), 0.1), -90));
    new JoystickButton(driver, Button.kY.value).whileActiveContinuous(new TurnToAngle(drivetrainSubsystem,
        () -> Utils.deadband(driver.getRightX(), 0.1), () -> Utils.deadband(driver.getRightY(), 0.1), 0));

    new JoystickButton(driver, Button.kLeftBumper.value).whileActiveContinuous(new AimContinuously(drivetrainSubsystem, climbSubsystem, turretSubsystem,
        () -> Utils.deadband(driver.getRightX(), 0.1), () -> Utils.deadband(driver.getRightY(), 0.1)));

    // OPERATOR CONTROLS

    // A Button activates current climb state. The activate climb state checks if
    // the turret is in the correct position.
    new JoystickButton(operator, Button.kA.value)
        .whileActiveContinuous(new ActivateClimbState(climbSubsystem, turretSubsystem));
    new JoystickButton(operator, Button.kA.value)
        .whenPressed(() -> turretSubsystem.setDefaultCommand(new DescheduleSubsystem(turretSubsystem)));

    // Left Bumper decreases climb state
    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(() -> climbSubsystem.previousState());

    // Right Bumper increases climb state
    new JoystickButton(operator, Button.kRightBumper.value).whenPressed(() -> climbSubsystem.nextState());

    // X Button holds balls
    new JoystickButton(operator, Button.kX.value).whileActiveContinuous(new HoldBalls(indexerSubsystem));

    // B Button deploys intake and runs intake and indexer to the hold ball position
    new JoystickButton(operator, Button.kB.value).whileActiveContinuous(new Intake(indexerSubsystem, intakeSubsystem));

    // Y Button starts shooter
    new JoystickButton(operator, Button.kY.value).whileActiveContinuous(new Shoot(shooterSubsystem, indexerSubsystem));
    new JoystickButton(operator, Button.kA.value)
        .whenPressed(() -> turretSubsystem.setDefaultCommand(new AlignTurret(turretSubsystem, climbSubsystem)));

    // Start Button runs indexer backwards to clear shooter
    new JoystickButton(operator, Button.kStart.value).whileActiveContinuous(new Deload(indexerSubsystem));

    // Back Button moves turret to eject position and ejects balls through shooter
    new JoystickButton(operator, Button.kBack.value)
        .whileActiveContinuous(new EjectBall(turretSubsystem, shooterSubsystem, indexerSubsystem));
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
