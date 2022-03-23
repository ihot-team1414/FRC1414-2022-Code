// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.HoodAutoCommand;
import frc.robot.commands.IndexerAutoCommand;
import frc.robot.commands.IntakeAutoCommand;
import frc.robot.commands.IntakeAutoDeployCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveClimbCommand;
import frc.robot.commands.RobotStartCommand;
import frc.robot.commands.ShooterAutoCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterEjectCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.TurretAutoCommand;
import frc.robot.commands.TurretEjectCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ArmPosition;
import frc.robot.subsystems.ClimbSubsystem.ElevatorPosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // Xbox Controllers
  private final XboxController driver = new XboxController(1);
  private final XboxController operator = new XboxController(0);

  public static double stickDeadband(final double value, final double deadband, final double center) {
    return (value < (center + deadband) && value > (center - deadband)) ? center : value;
  }

  private SendableChooser<Command> chooser = new SendableChooser<>();

  private final Pose2d startingPositions[] = {new Pose2d(8, 2.84, Rotation2d.fromDegrees(68)), new Pose2d(0, 0, Rotation2d.fromDegrees(0))};

  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem(startingPositions[1]);

  private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();

  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();

  private final IntakeSubsystem m_intakesubsystem = new IntakeSubsystem();

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // this.chooser.setDefaultOption("Wait Command", new WaitCommand(15));
    final TrapezoidProfile.Constraints kThetaControllerConstraints = //
    new TrapezoidProfile.Constraints(
            m_drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            m_drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      Constants.DRIVETRAIN_MAX_VEL,
      Constants.DRIVETRAIN_MAX_ACCELERATION)
              .setKinematics(m_drivetrain.kinematics);


    PIDController xController = new PIDController(Constants.DRIVETRAIN_PATH_X_KP, 0, 0);
    PIDController yController = new PIDController(Constants.DRIVETRAIN_PATH_Y_KP, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      Constants.DRIVETRAIN_PATH_THETA_KP, 0, 0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    // 2. Generate trajectory
    Trajectory outside4BallAutoTrajectories[] = {
      TrajectoryGenerator.generateTrajectory(
        startingPositions[0],
        List.of(
        ),
        new Pose2d(7.64, 0.72, Rotation2d.fromDegrees(90)),
        trajectoryConfig
      ),
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(7.64, 0.72, Rotation2d.fromDegrees(270)),
        List.of(
        ),
        new Pose2d(1.43, 1.40, Rotation2d.fromDegrees(45)),
        trajectoryConfig
      ),
    };

    

    // 4. Construct command to follow trajectory
    SwerveControllerCommand outside4BallAutoCommands[] = {
      new SwerveControllerCommand(
        outside4BallAutoTrajectories[0],
          () -> m_drivetrain.getPose(),
          m_drivetrain.kinematics,
          xController,
          yController,
          thetaController,
          m_drivetrain::setModuleStates,
          m_drivetrain
      ),
      new SwerveControllerCommand(
        outside4BallAutoTrajectories[1],
          () -> m_drivetrain.getPose(),
          m_drivetrain.kinematics,
          xController,
          yController,
          thetaController,
          m_drivetrain::setModuleStates,
          m_drivetrain
      ),
    };

    // this.chooser.addOption("2 Ball",  new SequentialCommandGroup(
    //   new IntakeAutoDeployCommand(m_intakesubsystem).withTimeout(0.5),
    //   new ParallelCommandGroup(
    //     new IntakeAutoCommand(m_intakesubsystem).withTimeout(3.5),
    //     new DriveStraightCommand(m_drivetrain, new Pose2d(1, 0, Rotation2d.fromDegrees(45))).withTimeout(3.5)
    //   ),
    //   new TurnToAngleCommand(m_drivetrain, 180).withTimeout(3),
    //   new ParallelCommandGroup(
    //     new InstantCommand(() -> m_intakesubsystem.setReverse(), m_intakesubsystem),
    //     new TurretAutoCommand(m_turretSubsystem),
    //     new HoodAutoCommand(m_hoodSubsystem),
    //     new ShooterAutoCommand(m_shooterSubsystem),
    //     new SequentialCommandGroup(
    //       new WaitCommand(2),
    //       new IndexerAutoCommand(m_indexersubsystem, () -> true)
    //     )
    //   ).withTimeout(6))
    // );

    // this.chooser.addOption("4 Ball Outside",  new SequentialCommandGroup(
    //   new IntakeAutoDeployCommand(m_intakesubsystem).withTimeout(0.5),
    //   new ParallelCommandGroup(
    //     new IntakeAutoCommand(m_intakesubsystem).withTimeout(2.5),
    //     outside4BallAutoCommands[0].withTimeout(2.5)
    //   ),
    //   new TurnToAngleCommand(m_drivetrain, 180).withTimeout(1),
    //   new ParallelCommandGroup(
    //     new InstantCommand(() -> m_intakesubsystem.setReverse(), m_intakesubsystem),
    //     new TurretAutoCommand(m_turretSubsystem),
    //     new HoodAutoCommand(m_hoodSubsystem),
    //     new ShooterAutoCommand(m_shooterSubsystem),
    //     new SequentialCommandGroup(
    //       new WaitCommand(2),
    //       new IndexerAutoCommand(m_indexersubsystem, () -> true)
    //     )
    //   ).withTimeout(4),
    //   new IntakeAutoDeployCommand(m_intakesubsystem).withTimeout(0.5),
    //   new ParallelCommandGroup(
    //     new IntakeAutoCommand(m_intakesubsystem).withTimeout(3.5),
    //     outside4BallAutoCommands[1].withTimeout(3.5)
    //   ),
    //   new TurnToAngleCommand(m_drivetrain, 180).withTimeout(1),
    //   new ParallelCommandGroup(
    //     new IntakeAutoCommand(m_intakesubsystem),
    //     new TurretAutoCommand(m_turretSubsystem),
    //     new HoodAutoCommand(m_hoodSubsystem),
    //     new ShooterAutoCommand(m_shooterSubsystem),
    //     new SequentialCommandGroup(
    //       new WaitCommand(1),
    //       new IndexerAutoCommand(m_indexersubsystem, () -> true)
    //     )
    //   ).withTimeout(3)
    // )
    // );

    this.chooser.setDefaultOption("Drive spin", 
      new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          List.of(
          ),
          new Pose2d(-1, 0, Rotation2d.fromDegrees(0)),
          trajectoryConfig
        ),
        () -> m_drivetrain.getPose(),
        m_drivetrain.kinematics,
        xController,
        yController,
        thetaController,
        m_drivetrain::setModuleStates,
        m_drivetrain
      )
    );

    SmartDashboard.putData("Auto Chooser", this.chooser);

    m_hoodSubsystem.setDefaultCommand(new HoodAutoCommand(m_hoodSubsystem));
    // m_turretSubsystem.setDefaultCommand(new TurretAutoCommand(m_turretSubsystem));

    // Configure the button bindings
    configureButtonBindings();

    m_climbSubsystem.setDefaultCommand(new RobotStartCommand(m_climbSubsystem));

    m_drivetrain.setDefaultCommand(new DriveCommand(
            m_drivetrain,
            () -> modifyAxis(driver.getRightY()), // Axes are flipped here on purpose
            () -> modifyAxis(driver.getRightX()),
            () -> modifyAxis(driver.getLeftX()),
            () -> driver.getRightBumper(),
            () -> driver.getLeftBumper()
    ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

private static double deadband(double value, double deadband) {
  if (Math.abs(value) > deadband) {
      if (value > 0.0) {
          return (value - deadband) / (1.0 - deadband);
      } else {
          return (value + deadband) / (1.0 - deadband);
      }
  } else {
      return 0.0;
  }
}

private static double modifyAxis(double value) {
  // Deadband
  value = deadband(value, 0.1);

  return value;
}

double setPoint = 0.25;
int currentState = 0;
  private void configureButtonBindings() {

    new JoystickButton(driver, Button.kStart.value).whenPressed(() -> this.m_drivetrain.zeroGyroscope());

    Command climbStates[] = {
      new MoveClimbCommand(m_climbSubsystem, ArmPosition.Vertical, ElevatorPosition.Neutral),
      new MoveClimbCommand(m_climbSubsystem, ArmPosition.Lifting, ElevatorPosition.FirstRung),
      new SequentialCommandGroup(
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Lifting, ElevatorPosition.Starting),
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Grabbing, ElevatorPosition.Starting)
      ),
      new SequentialCommandGroup(
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Grabbing, ElevatorPosition.Intermediate),
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Tilting, ElevatorPosition.Intermediate),
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Tilting, ElevatorPosition.Extended),
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Vertical, ElevatorPosition.Extended)
      ),
      new SequentialCommandGroup(
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Lifting, ElevatorPosition.Starting),
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Grabbing, ElevatorPosition.Starting)
      ),
      new SequentialCommandGroup(
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Grabbing, ElevatorPosition.Intermediate),
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Tilting, ElevatorPosition.Intermediate),
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Tilting, ElevatorPosition.Extended),
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Vertical, ElevatorPosition.Extended)
      ),
      new SequentialCommandGroup(
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Lifting, ElevatorPosition.Starting),
        new MoveClimbCommand(m_climbSubsystem, ArmPosition.Grabbing, ElevatorPosition.Starting)
      ),
    };

    ElevatorPosition elevatorStates[] = { 
      ElevatorPosition.Neutral, //
      ElevatorPosition.FirstRung, //
      ElevatorPosition.Starting, //
      ElevatorPosition.Starting, //
      ElevatorPosition.Intermediate,
      ElevatorPosition.Intermediate,
      ElevatorPosition.Extended,
      ElevatorPosition.Extended,
      ElevatorPosition.Intermediate,
      ElevatorPosition.Starting,
      ElevatorPosition.Starting,
      ElevatorPosition.Intermediate,
      ElevatorPosition.Intermediate,
      ElevatorPosition.Extended,
      ElevatorPosition.Extended,
      ElevatorPosition.Intermediate,
      ElevatorPosition.Starting,
      ElevatorPosition.Starting,
      ElevatorPosition.Intermediate,
      ElevatorPosition.Neutral,
    };

    ArmPosition armStates[] = {
      ArmPosition.Vertical,
      ArmPosition.Lifting,
      ArmPosition.Lifting,
      ArmPosition.Grabbing,
      ArmPosition.Grabbing,
      ArmPosition.Tilting,
      ArmPosition.Tilting,
      ArmPosition.Vertical,
      ArmPosition.Lifting,
      ArmPosition.Lifting,
      ArmPosition.Grabbing,
      ArmPosition.Grabbing,
      ArmPosition.Tilting,
      ArmPosition.Tilting,
      ArmPosition.Vertical,
      ArmPosition.Lifting,
      ArmPosition.Lifting,
      ArmPosition.Grabbing,
      ArmPosition.Grabbing,
      ArmPosition.Tilting,
    };


    // Driver buttons for turn to angle
    new JoystickButton(driver, Button.kA.value).whileActiveContinuous(()-> m_drivetrain.turnToAngle(180), m_drivetrain);
    new JoystickButton(driver, Button.kX.value).whileActiveContinuous(()-> m_drivetrain.turnToAngle(90), m_drivetrain);
    new JoystickButton(driver, Button.kB.value).whileActiveContinuous(()-> m_drivetrain.turnToAngle(-90), m_drivetrain);
    new JoystickButton(driver, Button.kY.value).whileActiveContinuous(()-> m_drivetrain.turnToAngle(0), m_drivetrain);


    // A Button activates current climb state
    // new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.m_climbSubsystem.setArmPosition(armStates[currentState]));
    // new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.m_climbSubsystem.setElevatorPosition(elevatorStates[currentState]));

    // new JoystickButton(operator, Button.kA.value).whenPressed(new MoveClimbCommand(m_climbSubsystem, armStates[currentState], elevatorStates[currentState]).withTimeout(3));


    new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.m_turretSubsystem.resetPosition());

    // Left Bumper decreases climb state
    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(() -> { 
      currentState--;
    } );

    // Right Bumper increases climb state
    new JoystickButton(operator, Button.kRightBumper.value).whenPressed(() -> {
      currentState++;
    });


      // X Button holds balls
      new JoystickButton(operator, Button.kX.value).whenPressed(() -> this.m_indexerSubsystem.holdBalls()).whenReleased(() -> this.m_indexerSubsystem.stop());


    // B Button deploys intake and runs intake and indexer to the hold ball position
    new JoystickButton(operator, Button.kB.value).whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> m_intakesubsystem.setForward()),
      new WaitCommand(0.5),
      new InstantCommand(() -> m_intakesubsystem.setIntakeSpeed(Constants.INTAKE_SPEED)
    ))).whenReleased(() -> {
      this.m_intakesubsystem.setReverse();
      this.m_intakesubsystem.stop();
    });

    new JoystickButton(operator, Button.kB.value).whenPressed(() -> this.m_indexerSubsystem.holdBalls()).whenReleased(() -> this.m_indexerSubsystem.stop());

    // Y Button starts shooter
    new JoystickButton(operator, Button.kY.value).whileActiveContinuous(new ShooterCommand(m_shooterSubsystem, m_indexerSubsystem));
    new JoystickButton(operator, Button.kY.value).whileActiveContinuous(() -> this.m_turretSubsystem.visionTargeting());

    // Start runs indexer backwards to clear shooter
    new JoystickButton(operator, Button.kStart.value).whenPressed(() -> this.m_indexerSubsystem.reverse()).whenReleased(() -> this.m_indexerSubsystem.stop());

    // Back ejects wrong color ball through shooter
    // new JoystickButton(operator, Button.kBack.value).whenPressed(
    //   new ParallelCommandGroup(
    //     new TurretEjectCommand(m_turretSubsystem),
    //     new ShooterEjectCommand(m_shooterSubsystem),
    //     new SequentialCommandGroup(
    //       new WaitCommand(1),
    //       new IndexerAutoCommand(m_indexersubsystem, () -> true)
    //     )
    //   ).withTimeout(2)
    // );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return this.chooser.getSelected();  
  }
}
