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

  private final DoubleSolenoid intakePiston = new DoubleSolenoid(24, Constants.PCM_TYPE,
      Constants.INTAKE_PISTON_FORWARD, Constants.INTAKE_PISTON_REVERSE);

  private final Compressor compressor = new Compressor(24, Constants.PCM_TYPE);

  public final TalonSRX frontIntakeMotor = new TalonSRX(Constants.INTAKE_ID);

  public static synchronized IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }

    return instance;
  }

  private IntakeSubsystem() {
    frontIntakeMotor.setNeutralMode(NeutralMode.Brake);
    frontIntakeMotor.setInverted(false);

    compressor.enableDigital();

    close();
  }

  public void open() {
    intakePiston.set(Constants.INTAKE_PISTON_OPEN);
  }

  public void close() {
    stop();
    intakePiston.set(Constants.INTAKE_PISTON_CLOSED);
  }

  public void stop() {
    set(0);
  }

  public void intake() {
    set(Constants.INTAKE_SPEED);
  }

  public void outtake() {
    set(-Constants.INTAKE_SPEED);
  }

  public void set(double speed) {
    this.frontIntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
  }
}