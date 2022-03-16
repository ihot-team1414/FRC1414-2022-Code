package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{

  private final DoubleSolenoid intakePiston = new DoubleSolenoid(24, Constants.PCM_TYPE,
  Constants.INTAKE_PISTON_FORWARD, Constants.INTAKE_PISTON_REVERSE);
  private Value pistonState = Constants.INTAKE_PISTON_CLOSED;
  private final Compressor compressor = new Compressor(24, Constants.PCM_TYPE);


  public final TalonSRX frontIntakeMotor = new TalonSRX(Constants.INTAKE_ID);





  
  public IntakeSubsystem(){
    this.frontIntakeMotor.setNeutralMode(NeutralMode.Brake);
    this.frontIntakeMotor.setInverted(true);
    this.compressor.disable();

  }

  


  public Value getPistonState() {
    return this.pistonState;
  }

  public void toggleIntake() {
    if (getPistonState() == Constants.INTAKE_PISTON_OPEN) {
      this.intakePiston.set(Constants.INTAKE_PISTON_CLOSED);
      this.pistonState = Constants.INTAKE_PISTON_CLOSED;
    } else if (getPistonState() == Constants.INTAKE_PISTON_CLOSED) {
      this.intakePiston.set(Constants.INTAKE_PISTON_OPEN);
      this.pistonState = Constants.INTAKE_PISTON_OPEN;
    }
  }

  public void setForward() {
    this.intakePiston.set(Constants.INTAKE_PISTON_OPEN);
    this.pistonState = Constants.INTAKE_PISTON_OPEN;
    // this.compressor.disable();
  }

  public void setReverse() {
    this.intakePiston.set(Constants.INTAKE_PISTON_CLOSED);
    this.pistonState = Constants.INTAKE_PISTON_CLOSED;
    // this.compressor.enableDigital();
  }

  public void intake() {
      this.setFrontIntakeSpeed(0.5);
  }

  public void outtake() {
    this.setFrontIntakeSpeed(-0.5);
  }

  public void stop() {
    this.setFrontIntakeSpeed(0);
  }

  public void setFrontIntakeSpeed(double speed) {
    this.frontIntakeMotor.set(ControlMode.PercentOutput, speed);
  }  

  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      super.periodic();

      if (getPistonState() == Constants.INTAKE_PISTON_OPEN) {
        SmartDashboard.putBoolean("pistons", true);
      } else if (getPistonState() == Constants.INTAKE_PISTON_CLOSED) {
        SmartDashboard.putBoolean("pistons", false);
      }
  }

}// END OF CLASS