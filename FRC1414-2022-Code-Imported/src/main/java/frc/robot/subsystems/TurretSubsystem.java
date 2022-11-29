package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Limelight;

public class TurretSubsystem extends SubsystemBase {
  private static TurretSubsystem instance;

  //Creates object reference to hardware
  private TalonSRX turretMotor = new TalonSRX(Constants.TURRET_MOTOR_ID);

  //Creates PID controller for the vision
  private PIDController visionController = new PIDController(Constants.TURRET_MOTOR_VISTION_kP,
    Constants.TURRET_MOTOR_VISTION_kI, Constants.TURRET_MOTOR_VISTION_kD);

  //Return single instance of IndexerSubsystem Class (Singleton)
  public static synchronized TurretSubsystem getInstance() {
    if (instance == null) {
      instance = new TurretSubsystem();
    }

    return instance;
  }

  //Constructor values are applied upon creation / reference of ShooterSubsystem object/class
  public TurretSubsystem() {
    //Make motor inverted
    turretMotor.setInverted(true);

    //Configure the feedback sensor / encoder
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configNominalOutputForward(0.0, 30);
    turretMotor.configNominalOutputReverse(0.0, 30);
    turretMotor.configPeakOutputForward(Constants.TURRET_MOTOR_MAX_OUTPUT, 30);
    turretMotor.configPeakOutputReverse(-Constants.TURRET_MOTOR_MAX_OUTPUT, 30);
    turretMotor.selectProfileSlot(0, 0);
    turretMotor.config_kF(0, Constants.TURRET_MOTOR_POSITION_kF, 0);
    turretMotor.config_kP(0, Constants.TURRET_MOTOR_POSITION_kP, 0);
    turretMotor.config_kI(0, Constants.TURRET_MOTOR_POSITION_kI, 0);
    turretMotor.config_kD(0, Constants.TURRET_MOTOR_POSITION_kD, 0);
    turretMotor.configAllowableClosedloopError(0, Constants.TURRET_POSITION_ALLOWED_ERROR, 30);
    turretMotor.configForwardSoftLimitThreshold(Constants.TURRET_MAX_POS);
    turretMotor.configReverseSoftLimitThreshold(Constants.TURRET_MIN_POS);
    turretMotor.configReverseSoftLimitEnable(true);
    turretMotor.configForwardSoftLimitEnable(true);

    //Voltage compensation means that, 
    turretMotor.enableVoltageCompensation(true);
    turretMotor.configVoltageCompSaturation(12);
  }

  //Set turret speed
  public void setTurret(double speed) {
    turretMotor.set(ControlMode.PercentOutput, speed);
  }

  //Reset position to 0
  public void home() {
    setPosition(0);
  }

  //Set position of turret
  public void setPosition(double pos) {
    turretMotor.set(ControlMode.Position, pos);
  }

  //TODO
  public void zero() {
    turretMotor.setSelectedSensorPosition(0);
  }

  //TODO
  public void eject() {
    setTurret(4000);
  }

  //Check if turret is within turn limits
  public boolean isWithinLimits() {
    return turretMotor.getSelectedSensorPosition() > Constants.TURRET_MIN_POS && turretMotor.getSelectedSensorPosition() < Constants.TURRET_MAX_POS;
  }

  //TODO
  public boolean isHome() {
    return turretMotor.getSelectedSensorPosition() < Constants.TURRET_POSITION_ALLOWED_ERROR
      && turretMotor.getSelectedSensorPosition() > -Constants.TURRET_POSITION_ALLOWED_ERROR;
  }

  //TODO
  public boolean isWithinAllowedError() {
    return Math.abs(turretMotor.getClosedLoopError()) < Constants.TURRET_POSITION_ALLOWED_ERROR;
  }

  //TODO
  public boolean isWithinAllowedVisionError() {
    return Math.abs(Limelight.getInstance().getDeltaX()) < Constants.TURRET_VISION_ALLOWED_ERROR;
  }

  //Get position of turret motor
  public double getPosition() {
    return turretMotor.getSelectedSensorPosition();
  }

  //TODO
  public void visionTargeting() {
    double deltaX = Limelight.getInstance().getDeltaX();

    if ((Limelight.getInstance().detectsTarget())) {
      setTurret(visionController.calculate(deltaX, 0));
    }
  }

  //Periodically (20 ms) update the following values for smart dashboard
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Target", turretMotor.getClosedLoopTarget());
    SmartDashboard.putNumber("Turret Position", turretMotor.getSelectedSensorPosition());

    // SmartDashboard.putNumber("Turret Closed Loop Error", turretMotor.getClosedLoopError());
    SmartDashboard.putNumber("Turret Output Percent", turretMotor.getMotorOutputPercent());
  }
}