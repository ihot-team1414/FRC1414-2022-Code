package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  private static DrivetrainSubsystem instance;

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;


  private final AHRS gyroscope = new AHRS(I2C.Port.kMXP);

  private SwerveDriveOdometry odometry;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private PIDController thetaController = new PIDController(Constants.DRIVETRAIN_ROTATION_kP, 0, 0);

  public static synchronized DrivetrainSubsystem getInstance() {
    if (instance == null) {
      instance = new DrivetrainSubsystem();
    }

    return instance;
  }

  private DrivetrainSubsystem() {
    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
    frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
        Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
        Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
        Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

    frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
        shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
        Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
        Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

    backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
        Constants.BACK_LEFT_MODULE_STEER_MOTOR,
        Constants.BACK_LEFT_MODULE_STEER_ENCODER,
        Constants.BACK_LEFT_MODULE_STEER_OFFSET);

    backRightModule = Mk3SwerveModuleHelper.createFalcon500(
        shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
        Constants.BACK_RIGHT_MODULE_STEER_OFFSET);
  }

  public void setStartingPosition(Pose2d startingPosition) {
    odometry = new SwerveDriveOdometry(
      Constants.KINEMATICS,
      Rotation2d.fromDegrees(-gyroscope.getFusedHeading()),
      startingPosition
    );
  }

  public void zeroGyroscope() {
    // odometry.resetPosition(
    //   new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
    //   Rotation2d.fromDegrees(-gyroscope.getFusedHeading()));  

    gyroscope.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
   if (gyroscope.isMagnetometerCalibrated()) {
     return Rotation2d.fromDegrees(gyroscope.getFusedHeading());
   }

   return Rotation2d.fromDegrees(360.0 - gyroscope.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = Constants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DRIVETRAIN_MAX_VEL);

    frontLeftModule.set(states[0].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[3].angle.getRadians());

    odometry.update(Rotation2d.fromDegrees(-gyroscope.getFusedHeading()), 
      new SwerveModuleState(
        -this.frontLeftModule.getDriveVelocity(), 
        new Rotation2d(frontLeftModule.getSteerAngle())
      ),
      new SwerveModuleState(
        -this.frontRightModule.getDriveVelocity(), 
        new Rotation2d(frontRightModule.getSteerAngle())
      ),
      new SwerveModuleState(
        -this.backLeftModule.getDriveVelocity(), 
        new Rotation2d(backRightModule.getSteerAngle())
      ),
      new SwerveModuleState(
        -this.backRightModule.getDriveVelocity(), 
        new Rotation2d(backRightModule.getSteerAngle())
      )
    );

    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry Rot", odometry.getPoseMeters().getRotation().getDegrees());

  }

  public Rotation2d getRotation() {
    return odometry.getPoseMeters().getRotation();
  }

  public double getGyroAngle() {
    return Rotation2d.fromDegrees(-gyroscope.getFusedHeading()).getRadians();
  }

  public double getRequiredTurningSpeedForAngle(double angle) {
    thetaController.enableContinuousInput(-180, 180);

    double currentAngle = getRotation().getDegrees() % 180;
    double targetAngle = angle % 180;
    double speed = thetaController.calculate(currentAngle, targetAngle);
    return -speed;
  }

  public void turnToAngle(double angle) {
    var chassis = new ChassisSpeeds(0, 0, getRequiredTurningSpeedForAngle(angle));
    this.chassisSpeeds = chassis;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
}
