package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleMK4 {

  // TODO: Tune these PID values for your robot
  private static final double kDriveP = 5; // 15.0
  private static final double kDriveI = 0.01;
  private static final double kDriveD = 0.1;
  private static final double kDriveF = 0.2;

  private static final double kAngleP = 0.2;
  private static final double kAngleI = 0.0;
  private static final double kAngleD = 0.0;

  // CANCoder has 4096 ticks/rotation
  private static double kEncoderTicksPerRotation = 4096;

  private CANSparkMax driveMotor;
  private CANSparkMax angleMotor;
  private SparkMaxPIDController m_anglePIDController;
  private SparkMaxPIDController m_drivePIDController;


  private CANCoder canCoder;

  public SwerveModuleMK4(CANSparkMax driveMotor, CANSparkMax angleMotor, CANCoder canCoder, Rotation2d offset) {
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.canCoder = canCoder;

    TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();
    angleMotor.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_anglePIDController = angleMotor.getPIDController();
  
    /**
     * The PID Controller can be configured to use the analog sensor as its feedback
     * device with the method SetFeedbackDevice() and passing the PID Controller
     * the CANAnalog object. 
     */

    // PID coefficients
    

    // set PID coefficients
    m_anglePIDController.setP(kAngleP);
    m_anglePIDController.setI(kAngleI);
    m_anglePIDController.setD(kAngleD);
    m_anglePIDController.setIZone(0);
    m_anglePIDController.setFF(0);
    m_anglePIDController.setOutputRange(-1, 1);



    m_drivePIDController = driveMotor.getPIDController();
  
    /**
     * The PID Controller can be configured to use the analog sensor as its feedback
     * device with the method SetFeedbackDevice() and passing the PID Controller
     * the CANAnalog object. 
     */

    // PID coefficients
    

    // set PID coefficients
    m_drivePIDController.setP(kDriveP);
    m_drivePIDController.setI(kDriveI);
    m_drivePIDController.setD(kDriveD);
    m_drivePIDController.setIZone(0);
    m_drivePIDController.setFF(0);
    m_drivePIDController.setOutputRange(-1, 1);
    

    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
    canCoder.configAllSettings(canCoderConfiguration);
  }

  public double getAngleOffset() {
    return canCoder.getAbsolutePosition();
  }

  /**
   * Gets the relative rotational position of the module
   * @return The relative rotational position of the angle motor in degrees
   */
  public Rotation2d getAngle() {
    // Note: This assumes the CANCoders are setup with the default feedback coefficient
    // and the sesnor value reports degrees.
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
  }

  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * @param desiredState - A SwerveModuleState representing the desired new state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);

    // Find the new absolute position of the module based on the difference in rotation
    double deltaTicks = (rotationDelta.getDegrees() / 360) * kEncoderTicksPerRotation;
    // Convert the CANCoder from it's position reading back to ticks
    double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
    double desiredTicks = currentTicks + deltaTicks;
    m_anglePIDController.setReference(desiredTicks, ControlType.kPosition);

    driveMotor.set(state.speedMetersPerSecond / (5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI));
  }



}