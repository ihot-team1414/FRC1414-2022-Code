package frc.robot.subsystems;


import java.security.GuardedObject;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.RollingAverage;

public class ShooterSubsystem extends SubsystemBase{

  public final TalonFX shooterMotor1 = new TalonFX(Constants.SHOOTER_ID_1);
  public final TalonFX shooterMotor2 = new TalonFX(Constants.SHOOTER_ID_2);
  
  

  // PID coefficients
  double kP = 0.001; 
  double kI = 0;
  double kD = 0; 
  double kIz = 0; 
  double kFF = 0.000156; 
  double kMaxOutput = 1; 
  double kMinOutput = -1;
  double maxRPM = 6000;
  double maxVel = 2000; // rpm
  double maxAcc = 1500;

  public ShooterSubsystem() {
    shooterMotor1.selectProfileSlot(0, 0);

    shooterMotor1.config_kF(0, kFF, 0);
    shooterMotor1.config_kP(0, kP, 0);
    shooterMotor1.config_kI(0, kI, 0);
    shooterMotor1.config_kD(0, kD, 0);
    shooterMotor1.configPeakOutputForward(0.75);
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

  // Calculating Distance in Meters
  public double calculateDistance() {
    double height = Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT;

    return height / Math.tan(Math.toRadians(calculateVisionAngle() + Constants.LIMELIGHT_Y_ANGLE));
  }

  public double calculateEntryAngle(double distance) {
    if(distance > 2 && distance < 4) {
      return Math.toRadians(-60);
    } else if (distance > 4 && distance < 8) {
      return Math.toRadians(-50);
    } else {
      return Math.toRadians(-80);
    }
  }

  public double calculateLaunchAngle(double distance, double entryAngle) {
    double height = Constants.TARGET_HEIGHT;

    return Math.atan((Math.tan(entryAngle)*distance-2*height)/-distance);
  }

  public double calculateLinearSpeed(double distance, double launchAngle, double entryAngle) {
    double height = Constants.TARGET_HEIGHT;
    return Math.sqrt(-((9.8*Math.pow(distance, 1)*(1+Math.pow(launchAngle, 2))/(2*height-2*distance*Math.tan(launchAngle)))));
  }

  public double calculateAngularSpeed(double linearSpeed) {
    double flywheelRadius = Constants.FLYWHEEL_RADIUS;
    return (linearSpeed*60)/(flywheelRadius*2*Math.PI);
  }

  public double calculateRPM(double angularSpeed) {
    return angularSpeed * 24/18;
  }

  public double getShootingSpeed() {

    return 330000 + 29000 * calculateDistance();

  }

  public void shoot() {
      shooterMotor1.set(ControlMode.Velocity, 330000 + 29000 * calculateDistance());
      SmartDashboard.putNumber("Shooter Speed", 330000 + 29000 * calculateDistance());
  }


  public void shootMaxRPM() {

    shooterMotor1.set(ControlMode.PercentOutput, 1);
  }

  public void stopShooting() {

    shooterMotor1.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Target Shooter Speed", 6500);
    SmartDashboard.putNumber("Actual Speed", this.shooterMotor1.getSelectedSensorVelocity());
  }

}// end of clas