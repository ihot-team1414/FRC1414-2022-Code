package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final DoubleSolenoid intakePiston = new DoubleSolenoid(24, Constants.PCM_TYPE,
      Constants.INTAKE_PISTON_FORWARD, Constants.INTAKE_PISTON_REVERSE);

  public final TalonSRX frontIntakeMotor = new TalonSRX(Constants.INTAKE_ID);

  public IntakeSubsystem() {
    frontIntakeMotor.setNeutralMode(NeutralMode.Brake);
    frontIntakeMotor.setInverted(true);

    close();
  }

  public void open() {
    intakePiston.set(Constants.INTAKE_PISTON_OPEN);
  }

  public void close() {
    this.stop();
    intakePiston.set(Constants.INTAKE_PISTON_CLOSED);
  }

  public void stop() {
    set(0);
  }

  public void set(double speed) {
    this.frontIntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
  }
}