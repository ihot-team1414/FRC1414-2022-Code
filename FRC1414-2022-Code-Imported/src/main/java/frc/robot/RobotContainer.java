// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveAutoCommand;
import frc.robot.commands.DriveAutoCommand2;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.HoodAutoCommand;
import frc.robot.commands.IndexerAutoCommand;
import frc.robot.commands.IntakeAutoCommand;
import frc.robot.commands.IntakeAutoDeployCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RobotStartCommand;
import frc.robot.commands.ShooterAutoCommand;
import frc.robot.commands.ShooterEjectCommand;
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

  private final Pose2d startingPositions[] = {new Pose2d(6.15, 2.35, Rotation2d.fromDegrees(25)), new Pose2d(0, 0, Rotation2d.fromDegrees(0))};

  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem(startingPositions[1]);

  private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();

  private final IndexerSubsystem m_indexersubsystem = new IndexerSubsystem();

  private final IntakeSubsystem m_intakesubsystem = new IntakeSubsystem();

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // this.chooser.setDefaultOption("Wait Command", new WaitCommand(15));
    // this.chooser.addOption("Drive/Intake Inside",  new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     new DriveAutoCommand(m_drivetrain, new Pose2d(5.31, 1.99, Rotation2d.fromDegrees(25))),
    //     new IntakeAutoCommand(m_intakesubsystem),
    //     new IndexerAutoCommand(m_indexersubsystem, () -> false)
    //   )
    // ));
    // this.chooser.addOption("2 Ball Inside",  new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     new DriveAutoCommand(m_drivetrain, new Pose2d(5.31, 1.99, Rotation2d.fromDegrees(25))),
    //     new IntakeAutoCommand(m_intakesubsystem),
    //     new IndexerAutoCommand(m_indexersubsystem, () -> false)
    //   ).withTimeout(5),
    //   new ParallelCommandGroup(
    //     new DriveAutoCommand(m_drivetrain, new Pose2d(5.31, 1.99, Rotation2d.fromDegrees(205))),
    //     new TurretAutoCommand(m_turretSubsystem),
    //     new HoodAutoCommand(m_hoodSubsystem),
    //     new ShooterAutoCommand(m_shooterSubsystem),
    //     new IndexerAutoCommand(m_indexersubsystem, () -> true)
    //   )
    // ));


    // this.chooser.addOption("4 Ball Inside",  new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     new DriveAutoCommand(m_drivetrain, new Pose2d(5.31, 1.99, Rotation2d.fromDegrees(25))),
    //     new IntakeAutoCommand(m_intakesubsystem),
    //     new IndexerAutoCommand(m_indexersubsystem, () -> false)
    //   ).withTimeout(2),
    //   new ParallelCommandGroup(
    //     new DriveAutoCommand(m_drivetrain, new Pose2d(5.31, 1.99, Rotation2d.fromDegrees(140))),
    //     new TurretAutoCommand(m_turretSubsystem),
    //     new HoodAutoCommand(m_hoodSubsystem),
    //     new ShooterAutoCommand(m_shooterSubsystem),
    //     new SequentialCommandGroup(
    //       new WaitCommand(1),
    //       new IndexerAutoCommand(m_indexersubsystem, () -> true)
    //     )
    //   ).withTimeout(4),
    //   new ParallelCommandGroup(
    //     new DriveAutoCommand(m_drivetrain, new Pose2d(1.44, 1.44, Rotation2d.fromDegrees(45))),
    //     new IntakeAutoCommand(m_intakesubsystem),
    //     new IndexerAutoCommand(m_indexersubsystem, () -> false)
    //   ).withTimeout(4),
    //   new ParallelCommandGroup(
    //     new IntakeAutoCommand(m_intakesubsystem), 
    //     new IndexerAutoCommand(m_indexersubsystem, () -> false)
    //   ).withTimeout(2),
    //   new ParallelCommandGroup(
    //     new DriveAutoCommand(m_drivetrain, new Pose2d(1.44, 1.44, Rotation2d.fromDegrees(120))),
    //     new TurretAutoCommand(m_turretSubsystem),
    //     new HoodAutoCommand(m_hoodSubsystem),
    //     new ShooterAutoCommand(m_shooterSubsystem),
    //     new SequentialCommandGroup(
    //       new WaitCommand(1),
    //       new IndexerAutoCommand(m_indexersubsystem, () -> true)
    //     )
    //   ).withTimeout(3))
    // );

    this.chooser.setDefaultOption("Drive/Intake Inside",  new SequentialCommandGroup(
      new IntakeAutoDeployCommand(m_intakesubsystem).withTimeout(0.5),
      new ParallelCommandGroup(
        new IntakeAutoCommand(m_intakesubsystem).withTimeout(3.5),
        new DriveAutoCommand(m_drivetrain, new Pose2d(1, 0, Rotation2d.fromDegrees(45))).withTimeout(3.5)
      ),
      new DriveAutoCommand2(m_drivetrain).withTimeout(3),
      new ParallelCommandGroup(
        new InstantCommand(() -> m_intakesubsystem.setReverse(), m_intakesubsystem),
        new TurretAutoCommand(m_turretSubsystem),
        new HoodAutoCommand(m_hoodSubsystem),
        new ShooterAutoCommand(m_shooterSubsystem),
        new SequentialCommandGroup(
          new WaitCommand(2),
          new IndexerAutoCommand(m_indexersubsystem, () -> true)
        )
      ).withTimeout(6))
    );

    SmartDashboard.putData("Auto Chooser", this.chooser);

    m_hoodSubsystem.setDefaultCommand(new HoodAutoCommand(m_hoodSubsystem));
    // m_turretSubsystem.setDefaultCommand(new TurretAutoCommand(m_turretSubsystem));

    // Configure the button bindings
    configureButtonBindings();

    m_climbSubsystem.setDefaultCommand(new RobotStartCommand(m_climbSubsystem));

    m_drivetrain.setDefaultCommand(new DriveCommand(
            m_drivetrain,
            () -> modifyAxis(0.75*driver.getRightY()), // Axes are flipped here on purpose
            () ->modifyAxis(0.75*driver.getRightX()),
            () -> modifyAxis(0.75*driver.getLeftX())
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

    ElevatorPosition elevatorStates[] = { 
      ElevatorPosition.Neutral,
      ElevatorPosition.FirstRung,
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
      ArmPosition.Starting,
      ArmPosition.Starting,
      ArmPosition.Grabbing,
      ArmPosition.Grabbing,
      ArmPosition.Tilting,
      ArmPosition.Tilting,
      ArmPosition.Vertical,
      ArmPosition.Starting,
      ArmPosition.Starting,
      ArmPosition.Grabbing,
      ArmPosition.Grabbing,
      ArmPosition.Tilting,
      ArmPosition.Tilting,
      ArmPosition.Vertical,
      ArmPosition.Starting,
      ArmPosition.Starting,
      ArmPosition.Grabbing,
      ArmPosition.Grabbing,
      ArmPosition.Tilting,
    };


    // A Button activates current climb state
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.m_climbSubsystem.setArmPosition(armStates[currentState]));
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.m_climbSubsystem.setElevatorPosition(elevatorStates[currentState]));
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.m_turretSubsystem.resetPosition());

    // Left Bumper decreases climb state
    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(() -> { 
      currentState--;
    } );

    // Right Bumper increases climb state
    new JoystickButton(operator, Button.kRightBumper.value).whenPressed(() -> {
      currentState++;
    });


    // B Button deploys intake and runs intake and indexer to the hold ball position
    new JoystickButton(operator, Button.kB.value).whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> m_intakesubsystem.setForward()),
      new WaitCommand(0.5),
      new InstantCommand(() -> m_intakesubsystem.setIntakeSpeed(Constants.INTAKE_SPEED)
    ))).whenReleased(() -> {
      this.m_intakesubsystem.setReverse();
      this.m_intakesubsystem.stop();
    });

    new JoystickButton(operator, Button.kB.value).whenPressed(() -> this.m_indexersubsystem.holdBalls()).whenReleased(() -> this.m_indexersubsystem.stop());

    // Y Button starts shooter
    new JoystickButton(operator, Button.kY.value).whenPressed(() -> this.m_shooterSubsystem.shoot()).whenReleased(()->this.m_shooterSubsystem.stop());   
    new JoystickButton(operator, Button.kY.value).whileActiveContinuous(() -> this.m_turretSubsystem.visionTargeting());

    // X Button loads ball into shooter
    new JoystickButton(operator, Button.kX.value).whenPressed(() -> this.m_indexersubsystem.load()).whenReleased(() -> { 
      this.m_indexersubsystem.stop();
    });

    // Start runs indexer backwards to clear shooter
    new JoystickButton(operator, Button.kStart.value).whenPressed(() -> this.m_indexersubsystem.reverse()).whenReleased(() -> this.m_indexersubsystem.stop());

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
