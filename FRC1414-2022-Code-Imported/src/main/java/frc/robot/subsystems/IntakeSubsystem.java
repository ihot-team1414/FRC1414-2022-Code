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
  public final TalonSRX frontIntakeMotor = new TalonSRX(Constants.INTAKE_ID);

  private final DoubleSolenoid frontIntakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,
  Constants.FRONT_INTAKE_FORWARD_ID, Constants.FRONT_INTAKE_RETRACTED_ID);

  private Value frontPistonState = Constants.INTAKE_PISTON_CLOSED;
  private Value backPistonState = Constants.INTAKE_PISTON_CLOSED;


  // private final Compressor compressor = new Compressor(Constants.PCM_ID);

  public IntakeSubsystem(){
    this.frontIntakeMotor.setNeutralMode(NeutralMode.Brake);
    this.frontIntakeMotor.setInverted(true);
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

  public Value getFrontPistonState() {
    return this.frontPistonState;
  }

  public void toggleFrontIntake() {
    if (getFrontPistonState() == Constants.INTAKE_PISTON_OPEN) {
      this.frontIntakePiston.set(Constants.INTAKE_PISTON_CLOSED);
      this.frontPistonState = Constants.INTAKE_PISTON_CLOSED;
    } else if (getFrontPistonState() == Constants.INTAKE_PISTON_CLOSED) {
      this.frontIntakePiston.set(Constants.INTAKE_PISTON_OPEN);
      this.frontPistonState = Constants.INTAKE_PISTON_OPEN;
    }
  }

  public void setFrontPistonForward() {
    this.frontIntakePiston.set(Constants.INTAKE_PISTON_OPEN);
    this.frontPistonState = Constants.INTAKE_PISTON_OPEN;
  }

  public void setFrontPistonReverse() {
    this.frontIntakePiston.set(Constants.INTAKE_PISTON_CLOSED);
    this.frontPistonState = Constants.INTAKE_PISTON_CLOSED;
  }

  

}// END OF CLASS