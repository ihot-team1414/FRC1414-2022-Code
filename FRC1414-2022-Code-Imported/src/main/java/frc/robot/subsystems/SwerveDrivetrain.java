// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;
import frc.robot.util.RollingAverage;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {
  private static final double TRACKWIDTH = 0.61; // in meters
  private static final double WHEELBASE = 0.61; // in meters
  public static final double kMaxSpeed = Units.feetToMeters(13.6); // 13.6 feet per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  public boolean fieldRelative;
  private RollingAverage avg = new RollingAverage();

    TalonFX frontLeftDriveMotor = new TalonFX(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR);
    TalonFX frontLeftSteeringMotor = new TalonFX(Constants.FRONT_LEFT_MODULE_STEER_MOTOR);


    TalonFX frontRightDriveMotor = new TalonFX(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
    TalonFX frontRightSteeringMotor = new TalonFX(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR);

    TalonFX backLeftDriveMotor = new TalonFX(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR);
    TalonFX backLeftSteeringMotor = new TalonFX(Constants.BACK_LEFT_MODULE_STEER_MOTOR);



    TalonFX backRightDriveMotor = new TalonFX(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR);
    TalonFX backRightSteeringMotor = new TalonFX(Constants.BACK_RIGHT_MODULE_STEER_MOTOR);

    AHRS gyro = new AHRS(I2C.Port.kMXP);
    // frontRightDriveMotor.setInverted(true);
    // backRightDriveMotor.setInverted(true);


  
  /**
   * TODO: These are example values and will need to be adjusted for your robot!
   * Modules are in the order of -
   * Front Left
   * Front Right
   * Back Left
   * Back Right
   *
   * Positive x values represent moving toward the front of the robot whereas
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics
    (
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );



  private final  SwerveModuleMK3 frontLeftModule = new SwerveModuleMK3(frontLeftDriveMotor, frontLeftSteeringMotor, new CANCoder(Constants.FRONT_LEFT_MODULE_STEER_ENCODER), Rotation2d.fromDegrees(Constants.FRONT_LEFT_MODULE_STEER_OFFSET)); // Front Left
  private final  SwerveModuleMK3 frontRightModule = new SwerveModuleMK3(frontRightDriveMotor,frontRightSteeringMotor, new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER),Rotation2d.fromDegrees(Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)); // Front Right
  private final  SwerveModuleMK3 backLeftModule = new SwerveModuleMK3( backLeftDriveMotor,  backLeftSteeringMotor, new CANCoder(Constants.BACK_LEFT_MODULE_STEER_ENCODER), Rotation2d.fromDegrees(Constants.BACK_LEFT_MODULE_STEER_OFFSET));// Back Left
  private final  SwerveModuleMK3 backRightModule= new SwerveModuleMK3(backRightDriveMotor, backRightSteeringMotor, new CANCoder(Constants.BACK_RIGHT_MODULE_STEER_ENCODER), Rotation2d.fromDegrees(Constants.BACK_RIGHT_MODULE_STEER_OFFSET));  // Back Right

  public SwerveModuleMK3[] modules = new SwerveModuleMK3[] {frontLeftModule, backLeftModule, frontRightModule , backRightModule, };


  public SwerveDrivetrain() {
    gyro.reset();
  }


  // public double getEncoderRawFrontLeftDrive() {
  //   return frontLeftDriveMotor.getSelectedSensorPosition(0);
  // }

  // public double getEncoderRawLeftAngle() {
  //   return frontLeftSteeringMotor.getSelectedSensorPosition();
  // }
  // public double getEncoderRawFrontRightDrive(){
  //   return frontRightDriveMotor.getSelectedSensorPosition();
  // }

  // public double getEncoderRawBackRightDrive(){
  //   return backRightDriveMotor.getSelectedSensorPosition();
  // }

  // public double getEncoderRawBackLeftDrive(){
  //   return backLeftDriveMotor.getSelectedSensorPosition();
  // }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    this.fieldRelative = fieldRelative;
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed);
    for (int i = 0; i < states.length; i++) {
      SwerveModuleMK3 module = modules[i];
      SwerveModuleState state = states[i];
      module.setDesiredState(state);
    }
  }
  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void resetGyro() {
    gyro.reset();

  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Left Module Angle", frontLeftModule.getAngleOffset());
    SmartDashboard.putNumber("Front Right Module Angle", frontRightModule.getAngleOffset());
    SmartDashboard.putNumber("Back Left Module Angle", backLeftModule.getAngleOffset());
    SmartDashboard.putNumber("Back Right Module Angle", backRightModule.getAngleOffset());
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    SmartDashboard.putBoolean("Field Relative?", fieldRelative);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
