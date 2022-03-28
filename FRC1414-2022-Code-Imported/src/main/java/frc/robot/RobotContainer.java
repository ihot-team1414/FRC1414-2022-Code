package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.FiveBallAuto;
import frc.robot.autos.TwoBallAuto;
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

  private final FiveBallAuto fiveBallAuto = new FiveBallAuto(
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

  TrajectoryConfig config;

  public RobotContainer() {
    // AUTO CHOOSER

    ArrayList<Translation2d> list = new ArrayList<>();
    list.add(new Translation2d(6, 4.75));
    list.add(new Translation2d(7, 5.25));

    SmartDashboard.putData("Auto Chooser", this.chooser);

    // chooser.addOption("Wait", new WaitCommand(15));
    chooser.addOption("5 Ball", fiveBallAuto.getAuto());

    // chooser.addOption("4 Ball Outside", fourBallAuto.getAuto());
    chooser.addOption("2 Ball", twoBallAuto.getAuto());

    chooser.addOption("Drive Straight", new DriveStraightOpenLoop(drivetrainSubsystem).withTimeout(3.5));

    chooser.addOption("Wait", new WaitCommand(15));

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

    new JoystickButton(driver, Button.kLeftBumper.value)
        .whileActiveContinuous(new AimContinuously(drivetrainSubsystem, climbSubsystem, turretSubsystem,
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
