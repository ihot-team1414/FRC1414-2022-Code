
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  private final TalonSRX loaderFrontMotor = new TalonSRX(Constants.LOADER_FRONT_MOTOR_ID);
  private final TalonSRX loaderBackMotor = new TalonSRX(Constants.LOADER_BACK_MOTOR_ID);
  private final TalonSRX funnelMotor = new TalonSRX(Constants.FUNNEL_MOTOR_ID);
  // private final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kMXP);
  private double startTime;

  private double loadingSpeed = 0.85;
  private double holdingSpeed = 0.75;
  private double funnelSpeed = 0.25;


  public IndexerSubsystem() {
    this.loaderFrontMotor.setNeutralMode(NeutralMode.Brake);
    this.loaderBackMotor.setNeutralMode(NeutralMode.Brake);
    this.funnelMotor.setNeutralMode(NeutralMode.Coast);
    this.loaderFrontMotor.setInverted(true);    
    this.loaderBackMotor.setInverted(true);
    startTime = Timer.getFPGATimestamp();
  }

  public void loadShooter() {
    this.loaderBackMotor.set(ControlMode.PercentOutput, loadingSpeed);
    this.loaderFrontMotor.set(ControlMode.PercentOutput, loadingSpeed);
  }

  public void holdBall() {
    this.loaderBackMotor.set(ControlMode.PercentOutput, -holdingSpeed);
    this.loaderFrontMotor.set(ControlMode.PercentOutput, holdingSpeed);  
  }

  public void stopLoader() {
    this.loaderBackMotor.set(ControlMode.PercentOutput, 0.0);
    this.loaderFrontMotor.set(ControlMode.PercentOutput, 0.0);
    this.funnelMotor.set(ControlMode.PercentOutput, 0.0);

  }

  public void funnel() {
    this.funnelMotor.set(ControlMode.PercentOutput, funnelSpeed);
  }

  public void eject() {
    this.funnelMotor.set(ControlMode.PercentOutput, -funnelSpeed);
    this.loaderBackMotor.set(ControlMode.PercentOutput, -loadingSpeed);
    this.loaderFrontMotor.set(ControlMode.PercentOutput, -loadingSpeed);  
  }

  public void holdBalls() {
    this.funnel();
    this.holdBall();
  }

  public void shoot() {
    this.funnel();
    this.loadShooter();
  }

  public void alternateFunnel() {
    // double current = Timer.getFPGATimestamp();
    // SmartDashboard.putNumber("Elapsed Time", current - startTime);
    // if ((current - startTime) % 1.0 > 0.25) {
    //   this.funnel();
    // } else {
    //   this.eject();
    // }
  }


  @Override
  public void periodic() {
    
  }
}