// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.RollingAverage;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

  private final CANSparkMax frontLeftDriveMotor = new CANSparkMax(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax frontLeftAngleMotor = new CANSparkMax(Constants.FRONT_LEFT_MODULE_STEER_MOTOR, MotorType.kBrushless);
  private final CANCoder frontLeftAngleEncoder = new CANCoder(Constants.FRONT_LEFT_MODULE_STEER_ENCODER);

  private final CANSparkMax backLeftDriveMotor = new CANSparkMax(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backLeftAngleMotor = new CANSparkMax(Constants.BACK_LEFT_MODULE_STEER_MOTOR, MotorType.kBrushless);
  private final CANCoder backLeftAngleEncoder = new CANCoder(Constants.BACK_LEFT_MODULE_STEER_ENCODER);


  private final CANSparkMax frontRightDriveMotor = new CANSparkMax(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax frontRightAngleMotor = new CANSparkMax(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, MotorType.kBrushless);
  private final CANCoder frontRightAngleEncoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);


  private final CANSparkMax backRightDriveMotor = new CANSparkMax(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backRightAngleMotor = new CANSparkMax(Constants.BACK_RIGHT_MODULE_STEER_MOTOR, MotorType.kBrushless);
  private final CANCoder backRightAngleEncoder = new CANCoder(Constants.BACK_RIGHT_MODULE_STEER_ENCODER);

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  private final Pose2d startPos;
  private Pose2d m_pos;

  private final SwerveDriveOdometry m_odometry;

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Uncomment if you are using a NavX
 private final AHRS m_navx = new AHRS(I2C.Port.kMXP); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModuleMK4 m_frontLeftModule;
  // private final SwerveModuleMK4 m_frontRightModule;
  // private final SwerveModuleMK4 m_backLeftModule;
  // private final SwerveModuleMK4 m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem(Pose2d startPos) {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    this.startPos = startPos;
    m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), startPos);
    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXME Setup motor configuration
    // m_frontLeftModule = Mk3SwerveModuleHelper.createNeo(
    //         // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
    //         tab.getLayout("Front Left Module", BuiltInLayouts.kList)
    //                 .withSize(2, 4)
    //                 .withPosition(0, 0),
    //         // This can either be STANDARD or FAST depending on your gear configuration
    //         Mk3SwerveModuleHelper.GearRatio.STANDARD,
    //         // This is the ID of the drive motor
    //         FRONT_LEFT_MODULE_DRIVE_MOTOR,
    //         // This is the ID of the steer motor
    //         FRONT_LEFT_MODULE_STEER_MOTOR,
    //         // This is the ID of the steer encoder
    //         FRONT_LEFT_MODULE_STEER_ENCODER,
    //         // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
    //         FRONT_LEFT_MODULE_STEER_OFFSET
    // );

    // // We will do the same for the other modules
    // m_frontRightModule = Mk3SwerveModuleHelper.createNeo(
    //         tab.getLayout("Front Right Module", BuiltInLayouts.kList)
    //                 .withSize(2, 4)
    //                 .withPosition(2, 0),
    //         Mk3SwerveModuleHelper.GearRatio.STANDARD,
    //         FRONT_RIGHT_MODULE_DRIVE_MOTOR,
    //         FRONT_RIGHT_MODULE_STEER_MOTOR,
    //         FRONT_RIGHT_MODULE_STEER_ENCODER,
    //         FRONT_RIGHT_MODULE_STEER_OFFSET
    // );

    // m_backLeftModule = Mk3SwerveModuleHelper.createNeo(
    //         tab.getLayout("Back Left Module", BuiltInLayouts.kList)
    //                 .withSize(2, 4)
    //                 .withPosition(4, 0),
    //         Mk3SwerveModuleHelper.GearRatio.STANDARD,
    //         BACK_LEFT_MODULE_DRIVE_MOTOR,
    //         BACK_LEFT_MODULE_STEER_MOTOR,
    //         BACK_LEFT_MODULE_STEER_ENCODER,
    //         BACK_LEFT_MODULE_STEER_OFFSET
    // );

    // m_backRightModule = Mk3SwerveModuleHelper.createNeo(
    //         tab.getLayout("Back Right Module", BuiltInLayouts.kList)
    //                 .withSize(2, 4)
    //                 .withPosition(6, 0),
    //         Mk3SwerveModuleHelper.GearRatio.STANDARD,
    //         BACK_RIGHT_MODULE_DRIVE_MOTOR,
    //         BACK_RIGHT_MODULE_STEER_MOTOR,
    //         BACK_RIGHT_MODULE_STEER_ENCODER,
    //         BACK_RIGHT_MODULE_STEER_OFFSET
    // );

    // m_frontRightModule = new SwerveModuleMK4(frontRightDriveMotor, frontRightAngleMotor, frontRightAngleEncoder, Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);
    m_frontLeftModule = new SwerveModuleMK4(frontRightDriveMotor, frontLeftAngleMotor, frontLeftAngleEncoder, Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
    // m_backRightModule = new SwerveModuleMK4(backRightDriveMotor, backRightAngleMotor, backRightAngleEncoder, Constants.BACK_RIGHT_MODULE_STEER_OFFSET);
    // m_backLeftModule = new SwerveModuleMK4(backLeftDriveMotor, backLeftAngleMotor, backLeftAngleEncoder, Constants.BACK_LEFT_MODULE_STEER_OFFSET);

    resetGyro();
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXME Remove if you are using a Pigeon
//     m_pigeon.setFusedHeading(0.0);
    // FIXME Uncomment if you are using a NavX
   m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {

    // FIXME Uncomment if you are using a NavX
   if (m_navx.isMagnetometerCalibrated()) {
     // We will only get valid fused headings if the magnetometer is calibrated
     return Rotation2d.fromDegrees(m_navx.getFusedHeading());
   }

   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   return Rotation2d.fromDegrees(360.0 - m_navx.getYaw() + 90);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  private RollingAverage avg = new RollingAverage();

  public double getGyroAngle() {
    return m_navx.getAngle();
  }
  
  double kp = 0.0375;//0.03
  double ki = 0.0; //0.01
  double kd = 0.0;
  double lastHeadingError = 0.0;
  double errorAccumulated = 0.0;

  double ykp = 0.0375;//0.03
  double yki = 0.0; //0.01
  double ykd = 0.0;
  double lastYError = 0.0;
  double yErrorAccumulated = 0.0;

  double xkp = 0.0375;//0.03
  double xki = 0.0; //0.01
  double xkd = 0.0;
  double lastXError = 0.0;
  double xErrorAccumulated = 0.0;

  public double getRequiredTurningSpeedForAngle(double angle) {
    double error = angle - this.getGyroAngle();
    this.errorAccumulated += error * Constants.TIME_STEP;
    double speed = (kp * error) + (ki * this.errorAccumulated) + (kd * (error - this.lastHeadingError));
    this.lastHeadingError = error;

    return speed;
  }

  public ChassisSpeeds getRequiredDrivingSpeeds(Pose2d targetPosition) {
    double yError = targetPosition.getY() - m_odometry.getPoseMeters().getY();
    this.yErrorAccumulated += yError * Constants.TIME_STEP;
    double ySpeed = (ykp * yError) + (yki * this.yErrorAccumulated) + (ykd * (yError - this.lastYError));
    this.lastYError = yError;

    double xError = targetPosition.getX() - m_odometry.getPoseMeters().getX();
    this.xErrorAccumulated += xError * Constants.TIME_STEP;
    double xSpeed = (xkp * xError) + (xki * this.xErrorAccumulated) + (xkd * (xError - this.lastXError));
    this.lastXError = xError;

    double error = targetPosition.getRotation().getRadians() - this.getGyroscopeRotation().getRadians();
    this.errorAccumulated += error * Constants.TIME_STEP;
    double speed = (kp * error) + (ki * this.errorAccumulated) + (kd * (error - this.lastHeadingError));
    this.lastHeadingError = error;

    return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, speed, this.getGyroscopeRotation());
  }

  public void driveToPosition(Pose2d targetPos) {
    drive(getRequiredDrivingSpeeds(targetPos));
  }

  public void resetErrors() {
    this.lastHeadingError = 0.0;
    this.errorAccumulated = 0.0;
  }

  // limelight visionTargeting
  // public double requiredVisionTargetingSpeed() {
  //   return this.getRequiredTurningSpeedForAngle(calculateVisionAngle());
  // }

  // public double calculateVisionAngle() {
  //   NetworkTableInstance.getDefault().startClientTeam(1414);
  //   NetworkTableInstance.getDefault().startDSClient();
  //   edu.wpi.first.networktables.NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //   NetworkTableEntry tx = table.getEntry("tx");

  //   avg.add(this.getGyroAngle() - tx.getDouble(0.0));

  //   return avg.getAverage();
  // }

  // public int detectsTarget() {
  //   NetworkTableInstance.getDefault().startClientTeam(1414);
  //   NetworkTableInstance.getDefault().startDSClient();
  //   edu.wpi.first.networktables.NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //   NetworkTableEntry tv = table.getEntry("tv");

  //   return (int)tv.getDouble(0.0);
  // }

  // public void setVisionMode(boolean on) {
  //   NetworkTableInstance.getDefault().startClientTeam(1414);
  //   NetworkTableInstance.getDefault().startDSClient();
  //   edu.wpi.first.networktables.NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //   NetworkTableEntry camMode = table.getEntry("camMode");
  //   NetworkTableEntry ledMode = table.getEntry("ledMode");
  //   if (on) {
  //     camMode.setDouble(0);
  //     ledMode.setDouble(3);
  //   } else {
  //     camMode.setDouble(1);
  //     ledMode.setDouble(1);
  //   }
  // }

  public void resetGyro() {
    m_navx.reset();

  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    // SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    // m_frontLeftModule.setDesiredState(states[0]);
    // m_frontRightModule.setDesiredState(states[1]);
    // m_backLeftModule.setDesiredState(states[2]);
    // m_backRightModule.setDesiredState(states[3]);

    SmartDashboard.putNumber("Front Left Angle", m_frontLeftModule.getAngle().getDegrees());
    // SmartDashboard.putNumber("Front Right Angle", m_frontRightModule.getAngle().getDegrees());
    // SmartDashboard.putNumber("Back Left Angle", m_backLeftModule.getAngle().getDegrees());
    // SmartDashboard.putNumber("Back Right Angle", m_backRightModule.getAngle().getDegrees());

    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());

    var gyroAngle = getGyroscopeRotation();


    m_odometry.update(gyroAngle, states[0], states[1], states[2], states[3]);
    }
}
