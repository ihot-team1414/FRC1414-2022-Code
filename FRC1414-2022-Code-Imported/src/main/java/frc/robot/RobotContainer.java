package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.subsystems.ClimbSubsystem.ArmPosition;
import frc.robot.subsystems.ClimbSubsystem.ElevatorPosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.util.Utils;

public class RobotContainer {
  // Xbox Controllers
  private final XboxController driver = new XboxController(1);
  private final XboxController operator = new XboxController(0);

  private SendableChooser<Command> chooser = new SendableChooser<>();

  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem(Constants.STARTING_POSITIONS[1]);

  private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();

  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();

  private final IntakeSubsystem m_intakesubsystem = new IntakeSubsystem();

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  public RobotContainer() {
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

    SmartDashboard.putData("Auto Chooser", this.chooser);

    m_hoodSubsystem.setDefaultCommand(new HoodAutoCommand(m_hoodSubsystem));
    m_turretSubsystem.setDefaultCommand(new TurretAutoCommand(m_turretSubsystem));

    configureButtonBindings();

    m_climbSubsystem.setDefaultCommand(new RobotStartCommand(m_climbSubsystem));

    m_drivetrain.setDefaultCommand(new DriveCommand(
      m_drivetrain,
      () -> Utils.deadband(driver.getRightY(), 0.1),
      () -> Utils.deadband(driver.getRightX(), 0.1),
      () -> Utils.deadband(driver.getLeftX(), 0.1),
      () -> driver.getRightBumper(),
      () -> driver.getLeftBumper()
    ));
  }

double setPoint = 0.25;
int currentState = 0;
  private void configureButtonBindings() {
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

    // Drive button to zero gyroscope
    new JoystickButton(driver, Button.kStart.value).whenPressed(() -> this.m_drivetrain.zeroGyroscope());

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

    // Start Button runs indexer backwards to clear shooter
    new JoystickButton(operator, Button.kStart.value).whenPressed(() -> this.m_indexerSubsystem.reverse()).whenReleased(() -> this.m_indexerSubsystem.stop());
  }

  public Command getAutonomousCommand() {
    return this.chooser.getSelected();  
  }
}
