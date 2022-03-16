// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RobotStartCommand;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // Xbox Controllers
  private final XboxController driver = new XboxController(1);
  private final XboxController operator = new XboxController(0);

  public static double stickDeadband(final double value, final double deadband, final double center) {
    return (value < (center + deadband) && value > (center - deadband)) ? center : value;
  }

  private final Pose2d[] startingPositions = {new Pose2d(5.0, 13.5, new Rotation2d()), new Pose2d(5.0, 13.5, new Rotation2d())};

  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();

  private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();

  private final IndexerSubsystem m_indexersubsystem = new IndexerSubsystem();

  private final IntakeSubsystem m_intakesubsystem = new IntakeSubsystem();

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_climbSubsystem.setDefaultCommand(new RobotStartCommand(m_climbSubsystem));

    m_drivetrain.setDefaultCommand(new DriveCommand(
            m_drivetrain,
            () -> -modifyAxis(-driver.getRightY()), // Axes are flipped here on purpose
            () -> modifyAxis(driver.getRightX()),
            () -> -modifyAxis(-driver.getLeftX())
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

  // Square the axis
  value = Math.copySign(value * value, value) * 0.5;

  return value;
}
double setPoint = 0.25;
int currentState = 0;
  private void configureButtonBindings() {

    new JoystickButton(driver, Button.kStart.value).whenPressed(() -> this.m_drivetrain.zeroGyroscope());

    ElevatorPosition elevatorStates[] = { 
      ElevatorPosition.Neutral,
      ElevatorPosition.Extended,
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
 
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.m_climbSubsystem.setArmPosition(armStates[currentState]));
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.m_climbSubsystem.setElevatorPosition(elevatorStates[currentState]));
    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(() -> currentState--);
    new JoystickButton(operator, Button.kRightBumper.value).whenPressed(() -> currentState++);

    new JoystickButton(operator, Button.kStart.value).whenPressed(() -> this.m_intakesubsystem.setIntakeSpeed(-Constants.INTAKE_SPEED)).whenReleased(() -> this.m_intakesubsystem.stop());
    new JoystickButton(operator, Button.kX.value).whenPressed(() -> this.m_intakesubsystem.setIntakeSpeed(Constants.INTAKE_SPEED)).whenReleased(() -> this.m_intakesubsystem.stop());

    new JoystickButton(operator, Button.kBack.value).whenPressed(() -> this.m_intakesubsystem.toggleIntake());
    new JoystickButton(operator, Button.kX.value).whenPressed(() -> this.m_indexersubsystem.shoot()).whenReleased(() -> this.m_indexersubsystem.stopLoader());
    new JoystickButton(operator, Button.kStart.value).whenPressed(() -> this.m_indexersubsystem.eject()).whenReleased(() -> this.m_indexersubsystem.stopLoader());
 
    new JoystickButton(operator, Button.kY.value).whenPressed(() -> this.m_hoodSubsystem.visionTargeting());;
    

    new JoystickButton(operator, Button.kY.value).whenPressed(() -> this.m_shooterSubsystem.shoot()).whenReleased(()->this.m_shooterSubsystem.stopShooting());
    new JoystickButton(operator, Button.kY.value).whileActiveContinuous(() -> this.m_turretSubsystem.visionTargeting()).whenInactive(() -> m_turretSubsystem.moveTurret(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
