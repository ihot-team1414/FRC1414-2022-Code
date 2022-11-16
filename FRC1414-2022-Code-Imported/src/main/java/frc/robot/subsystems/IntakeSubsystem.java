package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;

  //Create object reference for piston
  private final DoubleSolenoid intakePiston = new DoubleSolenoid(24, Constants.PCM_TYPE,
      Constants.INTAKE_PISTON_FORWARD, Constants.INTAKE_PISTON_REVERSE);

  //Create object reference for compressor
  private final Compressor compressor = new Compressor(24, Constants.PCM_TYPE);

  //Create object reference for front intake motor
  public final TalonSRX frontIntakeMotor = new TalonSRX(Constants.INTAKE_ID);

  //Return single instance of IndexerSubsystem Class (Singleton)
  public static synchronized IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }

    return instance;
  }

  //Constructor values are applied upon creation / reference of ShooterSubsystem object/class
  private IntakeSubsystem() {
    frontIntakeMotor.setNeutralMode(NeutralMode.Brake);
    //Front intake motor is inverted
    frontIntakeMotor.setInverted(false);

    compressor.enableDigital();

    close();
  }

  //Piston is ejected 
  public void open() {
    //Piston is set forward
    intakePiston.set(Constants.INTAKE_PISTON_OPEN);
  }

  //Piston is pulled back in
  public void close() {
    //Motor stops moving
    stop();
    intakePiston.set(Constants.INTAKE_PISTON_CLOSED);
  }

  //Motor speed is set to 0
  public void stop() {
    set(0);
  }

  //Set motor speed to intake speed
  public void intake() {
    set(Constants.INTAKE_SPEED);
  }

  //Set motor speed to outtake speed (negative intake speed)
  public void outtake() {
    set(-Constants.INTAKE_SPEED);
  }

  //Set the motor speed 
  //@param speed is used as input for motor speed (in percentage)
  public void set(double speed) {
    this.frontIntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
  }
}