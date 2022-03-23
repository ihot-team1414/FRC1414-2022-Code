package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.util.Utils;

public class RobotContainer {
  // CONTROLLERS
  private final XboxController driver = new XboxController(1);
  private final XboxController operator = new XboxController(0);

  // SUBSYSTEMS
  private final DrivetrainSubsystem drivetrainSubsystem = 
      new DrivetrainSubsystem(Constants.STARTING_POSITIONS[1]);

  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();

  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  private final TurretSubsystem turretSubsystem = new TurretSubsystem();

  private int currentState = 0;

  // AUTOS
  private SendableChooser<Command> chooser = new SendableChooser<>();

  private final FourBallAuto fourBallAuto = new FourBallAuto(
      drivetrainSubsystem,
      intakeSubsystem,
      indexerSubsystem,
      shooterSubsystem,
      turretSubsystem,
      hoodSubsystem);

  private final TwoBallAuto twoBallAuto = new TwoBallAuto(
      drivetrainSubsystem,
      intakeSubsystem,
      indexerSubsystem,
      shooterSubsystem,
      turretSubsystem,
      hoodSubsystem);

  public RobotContainer() {
    // AUTO CHOOSER
    SmartDashboard.putData("Auto Chooser", this.chooser);
    chooser.addOption("Wait", new WaitCommand(15));
    chooser.addOption("4 Ball Outside", fourBallAuto.getAuto());
    chooser.addOption("2 Ball High", twoBallAuto.getAuto());

    // DEFAULT COMMANDS
    hoodSubsystem.setDefaultCommand(new HoodAutoCommand(hoodSubsystem));
    turretSubsystem.setDefaultCommand(new TurretAutoCommand(turretSubsystem));
    climbSubsystem.setDefaultCommand(new RobotStartCommand(climbSubsystem));
    drivetrainSubsystem.setDefaultCommand(new DriveCommand(
        drivetrainSubsystem,
        () -> Utils.deadband(driver.getRightY(), 0.1),
        () -> Utils.deadband(driver.getRightX(), 0.1),
        () -> Utils.deadband(driver.getLeftX(), 0.1),
        () -> driver.getRightBumper(),
        () -> driver.getLeftBumper()
    ));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Drive button to zero gyroscope
    new JoystickButton(driver, Button.kStart.value).whenPressed(() -> drivetrainSubsystem.zeroGyroscope());

    // Driver buttons for turn to angle
    new JoystickButton(driver, Button.kA.value).whileActiveContinuous(() -> drivetrainSubsystem.turnToAngle(180), drivetrainSubsystem);
    new JoystickButton(driver, Button.kX.value).whileActiveContinuous(() -> drivetrainSubsystem.turnToAngle(90), drivetrainSubsystem);
    new JoystickButton(driver, Button.kB.value).whileActiveContinuous(() -> drivetrainSubsystem.turnToAngle(-90), drivetrainSubsystem);
    new JoystickButton(driver, Button.kY.value).whileActiveContinuous(() -> drivetrainSubsystem.turnToAngle(0), drivetrainSubsystem);


    // A Button activates current climb state
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> climbSubsystem.activateState());
    
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> turretSubsystem.home());

    // Left Bumper decreases climb state
    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(() -> climbSubsystem.previousState());

    // Right Bumper increases climb state
    new JoystickButton(operator, Button.kRightBumper.value).whenPressed(() -> climbSubsystem.nextState());

    // X Button holds balls
    new JoystickButton(operator, Button.kX.value).whenPressed(() -> indexerSubsystem.holdBalls()).whenReleased(() -> indexerSubsystem.stop());

    // B Button deploys intake and runs intake and indexer to the hold ball position
    new JoystickButton(operator, Button.kB.value).whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> intakeSubsystem.open()),
      new WaitCommand(0.5),
      new InstantCommand(() -> intakeSubsystem.set(Constants.INTAKE_SPEED)
    ))).whenReleased(() -> {
      intakeSubsystem.close();
      intakeSubsystem.stop();
    });

    new JoystickButton(operator, Button.kB.value).whenPressed(() -> indexerSubsystem.holdBalls()).whenReleased(() -> indexerSubsystem.stop());

    // Y Button starts shooter
    new JoystickButton(operator, Button.kY.value).whileActiveContinuous(new ShooterCommand(shooterSubsystem, indexerSubsystem));
    new JoystickButton(operator, Button.kY.value).whileActiveContinuous(() -> turretSubsystem.visionTargeting());

    // Start Button runs indexer backwards to clear shooter
    new JoystickButton(operator, Button.kStart.value).whenPressed(() -> indexerSubsystem.reverse()).whenReleased(() -> indexerSubsystem.stop());
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();  
  }
}
