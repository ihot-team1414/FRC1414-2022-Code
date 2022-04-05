
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Shoot;

public class IndexerSubsystem extends SubsystemBase {
  private static IndexerSubsystem instance;

  private final TalonSRX loaderFrontMotor = new TalonSRX(Constants.LOADER_FRONT_MOTOR_ID);
  private final TalonSRX loaderBackMotor = new TalonSRX(Constants.LOADER_BACK_MOTOR_ID);
  private final TalonSRX funnelMotor = new TalonSRX(Constants.FUNNEL_MOTOR_ID);
  
  public static synchronized IndexerSubsystem getInstance() {
    if (instance == null) {
      instance = new IndexerSubsystem();
    }

    return instance;
  }

  private IndexerSubsystem() {
    loaderFrontMotor.setNeutralMode(NeutralMode.Brake);
    loaderBackMotor.setNeutralMode(NeutralMode.Brake);
    funnelMotor.setNeutralMode(NeutralMode.Coast);
    loaderFrontMotor.setInverted(true);    
    loaderBackMotor.setInverted(true);
  }

  public void loadShooter() {
    if (ShooterSubsystem.getInstance().getShooterSpeed() > Constants.SHOOTER_MIN_LOAD_SPEED) {
      loaderBackMotor.set(ControlMode.PercentOutput, Constants.LOADING_SPEED);
      loaderFrontMotor.set(ControlMode.PercentOutput, Constants.LOADING_SPEED);
    }
  }

  public void holdBall() {
    loaderBackMotor.set(ControlMode.PercentOutput, -Constants.HOLDING_SPEED);
    loaderFrontMotor.set(ControlMode.PercentOutput, Constants.HOLDING_SPEED);  
  }

  public void stop() {
    loaderBackMotor.set(ControlMode.PercentOutput, 0.0);
    loaderFrontMotor.set(ControlMode.PercentOutput, 0.0);
    funnelMotor.set(ControlMode.PercentOutput, 0.0);
  }
  
  public void funnel() {
    funnelMotor.set(ControlMode.PercentOutput, Constants.FUNNEL_SPEED);
  }

  public void reverse() {
    funnelMotor.set(ControlMode.PercentOutput, -Constants.OUTTAKE_FUNNEL_SPEED);
    loaderBackMotor.set(ControlMode.PercentOutput, -Constants.LOADING_SPEED);
    loaderFrontMotor.set(ControlMode.PercentOutput, -Constants.LOADING_SPEED);  
  }

  public void holdBalls() {
    funnel();
    holdBall();
  }

  public void load() {
    funnel();
    loadShooter();
  }
}