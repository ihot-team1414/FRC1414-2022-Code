package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{

  private final DoubleSolenoid intakePiston = new DoubleSolenoid(24, Constants.PCM_TYPE,
  Constants.INTAKE_PISTON_FORWARD, Constants.INTAKE_PISTON_REVERSE);
  private Value pistonState = Constants.INTAKE_PISTON_CLOSED;
  private final Compressor compressor = new Compressor(24, Constants.PCM_TYPE);

  public final TalonSRX frontIntakeMotor = new TalonSRX(Constants.INTAKE_ID);
  
  public IntakeSubsystem(){
    frontIntakeMotor.setNeutralMode(NeutralMode.Brake);
    frontIntakeMotor.setInverted(true);
    compressor.disable();
  }

  public Value getPistonState() {
    return pistonState;
  }

  public void toggleIntake() {
    if (getPistonState() == Constants.INTAKE_PISTON_OPEN) {
      intakePiston.set(Constants.INTAKE_PISTON_CLOSED);
      pistonState = Constants.INTAKE_PISTON_CLOSED;
    } else if (getPistonState() == Constants.INTAKE_PISTON_CLOSED) {
      intakePiston.set(Constants.INTAKE_PISTON_OPEN);
      pistonState = Constants.INTAKE_PISTON_OPEN;
    }
  }

  public void setForward() {
    intakePiston.set(Constants.INTAKE_PISTON_OPEN);
    pistonState = Constants.INTAKE_PISTON_OPEN;
  }

  public void setReverse() {
    intakePiston.set(Constants.INTAKE_PISTON_CLOSED);
    pistonState = Constants.INTAKE_PISTON_CLOSED;
  }

  public void stop() {
    setIntakeSpeed(0);
  }

  public void setIntakeSpeed(double speed) {
    this.frontIntakeMotor.set(ControlMode.PercentOutput, speed);
  }  

  @Override
  public void periodic() {
  }
}