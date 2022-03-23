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
import frc.robot.autos.*;
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

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(Constants.STARTING_POSITIONS[1]);

  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();

  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  private final TurretSubsystem turretSubsystem = new TurretSubsystem();

  private final FourBallAuto fourBallAuto = new FourBallAuto(drivetrainSubsystem, intakeSubsystem, indexerSubsystem, shooterSubsystem, turretSubsystem, hoodSubsystem);
  private final TwoBallAuto twoBallAuto = new TwoBallAuto(drivetrainSubsystem, intakeSubsystem, indexerSubsystem, shooterSubsystem, turretSubsystem, hoodSubsystem);

  public RobotContainer() {



    this.chooser.addOption("Wait Command", new WaitCommand(15));
    this.chooser.addOption("4 Ball Outside", fourBallAuto.getAuto());
    this.chooser.addOption("2 Ball High", twoBallAuto.getAuto());

    SmartDashboard.putData("Auto Chooser", this.chooser);

    hoodSubsystem.setDefaultCommand(new HoodAutoCommand(hoodSubsystem));
    turretSubsystem.setDefaultCommand(new TurretAutoCommand(turretSubsystem));

    configureButtonBindings();

    climbSubsystem.setDefaultCommand(new RobotStartCommand(climbSubsystem));

    drivetrainSubsystem.setDefaultCommand(new DriveCommand(
      drivetrainSubsystem,
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
    new JoystickButton(driver, Button.kStart.value).whenPressed(() -> this.drivetrainSubsystem.zeroGyroscope());

    // Driver buttons for turn to angle
    new JoystickButton(driver, Button.kA.value).whileActiveContinuous(()-> drivetrainSubsystem.turnToAngle(180), drivetrainSubsystem);
    new JoystickButton(driver, Button.kX.value).whileActiveContinuous(()-> drivetrainSubsystem.turnToAngle(90), drivetrainSubsystem);
    new JoystickButton(driver, Button.kB.value).whileActiveContinuous(()-> drivetrainSubsystem.turnToAngle(-90), drivetrainSubsystem);
    new JoystickButton(driver, Button.kY.value).whileActiveContinuous(()-> drivetrainSubsystem.turnToAngle(0), drivetrainSubsystem);


    // A Button activates current climb state
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.climbSubsystem.setArmPosition(armStates[currentState]));
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.climbSubsystem.setElevatorPosition(elevatorStates[currentState]));
    
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> this.turretSubsystem.resetPosition());

    // Left Bumper decreases climb state
    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(() -> { 
      currentState--;
    } );

    // Right Bumper increases climb state
    new JoystickButton(operator, Button.kRightBumper.value).whenPressed(() -> {
      currentState++;
    });

    // X Button holds balls
    new JoystickButton(operator, Button.kX.value).whenPressed(() -> this.indexerSubsystem.holdBalls()).whenReleased(() -> this.indexerSubsystem.stop());

    // B Button deploys intake and runs intake and indexer to the hold ball position
    new JoystickButton(operator, Button.kB.value).whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> intakeSubsystem.setForward()),
      new WaitCommand(0.5),
      new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(Constants.INTAKE_SPEED)
    ))).whenReleased(() -> {
      this.intakeSubsystem.setReverse();
      this.intakeSubsystem.stop();
    });

    new JoystickButton(operator, Button.kB.value).whenPressed(() -> this.indexerSubsystem.holdBalls()).whenReleased(() -> this.indexerSubsystem.stop());

    // Y Button starts shooter
    new JoystickButton(operator, Button.kY.value).whileActiveContinuous(new ShooterCommand(shooterSubsystem, indexerSubsystem));
    new JoystickButton(operator, Button.kY.value).whileActiveContinuous(() -> this.turretSubsystem.visionTargeting());

    // Start Button runs indexer backwards to clear shooter
    new JoystickButton(operator, Button.kStart.value).whenPressed(() -> this.indexerSubsystem.reverse()).whenReleased(() -> this.indexerSubsystem.stop());
  }

  public Command getAutonomousCommand() {
    return this.chooser.getSelected();  
  }
}
