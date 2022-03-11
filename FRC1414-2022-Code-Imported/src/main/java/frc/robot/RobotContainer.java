// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Xbox Controllers
  private final XboxController driver = new XboxController(1);
  private final XboxController operator = new XboxController(0);

  public static double stickDeadband(final double value, final double deadband, final double center) {
    return (value < (center + deadband) && value > (center - deadband)) ? center : value;
  }

  // The robot's subsystems and commands are defined here...

  private final Pose2d[] startingPositions = {new Pose2d(5.0, 13.5, new Rotation2d()), new Pose2d(5.0, 13.5, new Rotation2d())};

  // Subsystems
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(startingPositions[0]);


  private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
  // Commands


  private final DefaultDriveCommand m_driveCommand = new DefaultDriveCommand(m_drivetrainSubsystem,
    () -> -driver.getRightY(), 
    () -> -driver.getRightX(),
    () -> driver.getLeftX()
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    this.m_drivetrainSubsystem.setDefaultCommand(m_driveCommand);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(driver, Button.kStart.value).whenPressed(() -> this.m_drivetrainSubsystem.resetGyro());

    new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.m_hoodSubsystem.setAngle(25));
    new JoystickButton(operator, Button.kB.value).whenPressed(() -> this.m_hoodSubsystem.setAngle(35));
    new JoystickButton(operator, Button.kX.value).whenPressed(() -> this.m_hoodSubsystem.setAngle(50));
    new JoystickButton(operator, Button.kY.value).whenPressed(() -> this.m_hoodSubsystem.setAngle(60));


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
