package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
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
  private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
  private final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
  private final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
  private final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();

  // AUTOS
  private SendableChooser<Command> chooser = new SendableChooser<>();

  private final FiveBallAuto fiveBallAuto = new FiveBallAuto();

  private final TwoBallAuto twoBallAuto = new TwoBallAuto();

  TrajectoryConfig config;

  public RobotContainer() {
    // INITIALIZE REMAINING SUBSYSTEMS
    IndexerSubsystem.getInstance();
    IntakeSubsystem.getInstance();
    ShooterSubsystem.getInstance();

    // STARTING POSITION CONFIG
    drivetrainSubsystem.setStartingPosition(Constants.STARTING_POSITIONS[0]);

    // CAMERA
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(192, 108);
    camera.setExposureManual(20);
    camera.setExposureHoldCurrent();

    // AUTO CHOOSER
    SmartDashboard.putData("Auto Chooser", this.chooser);
    chooser.setDefaultOption("5 Ball", fiveBallAuto.getAuto());
    chooser.addOption("2 Ball", twoBallAuto.getAuto());
    chooser.addOption("Taxi", new DriveStraightOpenLoop().withTimeout(3.5));
    chooser.addOption("Wait", new WaitCommand(15));

    // DEFAULT COMMANDS
    hoodSubsystem.setDefaultCommand(new AlignHood());

    // The align turret command checks to see if the pivot arms are in the vertical position, otherwise, it homes.
    turretSubsystem.setDefaultCommand(new AlignTurret());

    drivetrainSubsystem.setDefaultCommand(new Drive(
        () -> Utils.deadband(driver.getRightY(), 0.1),
        () -> Utils.deadband(driver.getRightX(), 0.1),
        () -> Utils.deadband(driver.getLeftX(), 0.1),
        () -> (driver.getRightTriggerAxis() > 0.5),
        () -> (driver.getLeftTriggerAxis() > 0.5))
    );

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // DRIVER CONTROLS

    // Start button to zero gyroscope
    new JoystickButton(driver, Button.kStart.value).whenPressed(() -> drivetrainSubsystem.zeroGyroscope());

    // Driver buttons for turn to angle
    new JoystickButton(driver, Button.kA.value).whileActiveContinuous(new TurnToAngle(() -> Utils.deadband(driver.getRightX(), 0.1), () -> Utils.deadband(driver.getRightY(), 0.1), 180));
    new JoystickButton(driver, Button.kX.value).whileActiveContinuous(new TurnToAngle(() -> Utils.deadband(driver.getRightX(), 0.1), () -> Utils.deadband(driver.getRightY(), 0.1), 90));
    new JoystickButton(driver, Button.kB.value).whileActiveContinuous(new TurnToAngle(() -> Utils.deadband(driver.getRightX(), 0.1), () -> Utils.deadband(driver.getRightY(), 0.1), -90));
    new JoystickButton(driver, Button.kY.value).whileActiveContinuous(new TurnToAngle(() -> Utils.deadband(driver.getRightX(), 0.1), () -> Utils.deadband(driver.getRightY(), 0.1), 0));

    // Left bumper aims continuously
    new JoystickButton(driver, Button.kLeftBumper.value).whileActiveContinuous(new AimContinuously(() -> Utils.deadband(driver.getRightX(), 0.1), () -> Utils.deadband(driver.getRightY(), 0.1)));

    // Back button resets climb state
    new JoystickButton(driver, Button.kBack.value).whenPressed(() -> climbSubsystem.resetState());

    // OPERATOR CONTROLS

    // A Button activates current climb state. The activate climb state checks if the turret is in the correct position.
    new JoystickButton(operator, Button.kA.value).whileActiveContinuous(new ActivateClimbState());
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> turretSubsystem.setDefaultCommand(new DescheduleSubsystem(turretSubsystem)));
    new JoystickButton(operator, Button.kA.value).whenPressed(() -> climbSubsystem.setDefaultCommand(new DescheduleSubsystem(climbSubsystem)));

    // Left Bumper decreases climb state
    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(() -> climbSubsystem.previousState());

    // Right Bumper increases climb state
    new JoystickButton(operator, Button.kRightBumper.value).whenPressed(() -> climbSubsystem.nextState());

    // X Button shoots layup
    new JoystickButton(operator, Button.kX.value).whileActiveContinuous(new ShootLowGoal());

    // B Button deploys intake and runs intake and indexer to the hold ball position
    new JoystickButton(operator, Button.kB.value).whileActiveContinuous(new Intake());

    // Y Button starts shooter
    new JoystickButton(operator, Button.kY.value).whileActiveContinuous(new Shoot());
    new JoystickButton(operator, Button.kY.value).whileActiveContinuous(new AlignTurret());
    new JoystickButton(operator, Button.kY.value).whenPressed(() -> turretSubsystem.setDefaultCommand(new AlignTurret()));

    // Right stick homes turret
    new JoystickButton(operator, Button.kRightStick.value).whileActiveContinuous(new AlignTurretManually(() -> operator.getRightX()));

    // Start Button runs indexer backwards to clear shooter
    new JoystickButton(operator, Button.kStart.value).whileActiveContinuous(new Deload());

    // Back Button moves turret to eject position and ejects balls through shooter
    new JoystickButton(operator, Button.kBack.value).whileActiveContinuous(new EjectBall());
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public Command getTestCommand() {
    return new Spool();
  }
}
