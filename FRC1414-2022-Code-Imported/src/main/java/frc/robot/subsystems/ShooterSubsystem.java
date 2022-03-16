package frc.robot.subsystems;
import frc.robot.Constants;
import frc.util.RollingAverage;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

  public final TalonFX shooterMotor1 = new TalonFX(Constants.SHOOTER_ID_1);
  public final TalonFX shooterMotor2 = new TalonFX(Constants.SHOOTER_ID_2);

  public ShooterSubsystem() {
    shooterMotor1.selectProfileSlot(0, 0);

    shooterMotor1.config_kF(0, Constants.SHOOTER_kF, 0);
    shooterMotor1.config_kP(0, Constants.SHOOTER_kP, 0);
    shooterMotor1.config_kI(0, Constants.SHOOTER_kI, 0);
    shooterMotor1.config_kD(0, Constants.SHOOTER_kD, 0);
    shooterMotor1.configPeakOutputForward(Constants.SHOOTER_MAX_OUTPUT);
    shooterMotor2.follow(shooterMotor1);
    shooterMotor1.setInverted(true);
    shooterMotor1.setNeutralMode(NeutralMode.Coast);
    shooterMotor2.setNeutralMode(NeutralMode.Coast);
  }

  private RollingAverage avg = new RollingAverage();

  public double calculateVisionAngle() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");

    avg.add(ty.getDouble(0.0));

    return avg.getAverage();
  }

  public double calculateDistance() {
    double height = Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT;
    return height / Math.tan(Math.toRadians(calculateVisionAngle() + Constants.LIMELIGHT_Y_ANGLE));
  }

  public void shoot() {
      shooterMotor1.set(ControlMode.Velocity, 330000 + 29000 * calculateDistance());
      SmartDashboard.putNumber("Target Shooter Velocity", 330000 + 29000 * calculateDistance());
  }

  public void shootMaxRPM() {

    shooterMotor1.set(ControlMode.PercentOutput, 1);
  }

  public void stopShooting() {
    shooterMotor1.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity", this.shooterMotor1.getSelectedSensorVelocity());
  }
}