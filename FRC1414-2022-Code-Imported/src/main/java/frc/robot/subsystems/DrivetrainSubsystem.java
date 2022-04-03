package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Mk3SwerveModuleHelper;
import frc.lib.SdsModuleConfigurations;
import frc.lib.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
  private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200);

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    
    frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
            Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
            Constants.FRONT_LEFT_MODULE_STEER_OFFSET
    );

    frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
            Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
            Constants.FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.BACK_LEFT_MODULE_STEER_MOTOR,
            Constants.BACK_LEFT_MODULE_STEER_ENCODER,
            Constants.BACK_LEFT_MODULE_STEER_OFFSET
    );

    backRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
            Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
            Constants.BACK_RIGHT_MODULE_STEER_OFFSET
    );
  }

  public void zeroGyroscope() {
    navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
   if (navx.isMagnetometerCalibrated()) {
     return Rotation2d.fromDegrees(navx.getFusedHeading());
   }

   return Rotation2d.fromDegrees(360.0 - navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.DRIVETRAIN_MAX_VELOCITY_METERS_PER_SECOND);

    frontLeftModule.set(states[0].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VELOCITY_METERS_PER_SECOND * Constants.DRIVETRAIN_MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VELOCITY_METERS_PER_SECOND * Constants.DRIVETRAIN_MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VELOCITY_METERS_PER_SECOND * Constants.DRIVETRAIN_MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VELOCITY_METERS_PER_SECOND * Constants.DRIVETRAIN_MAX_VOLTAGE, states[3].angle.getRadians());
  }

  public Rotation2d getRotation() {
    return odometry.getPoseMeters().getRotation();
  }

  public double getGyroAngle() {
    return Rotation2d.fromDegrees(-gyroscope.getFusedHeading()).getRadians();
  }

  public double getRequiredTurningSpeedForAngle(double angle) {
    return getRequiredTurningSpeedForAngle(angle, false);
  }

  public double getRequiredTurningSpeedForAngle(double angle, boolean reverseRotation) {
    double currentAngle = getRotation().getDegrees() % 360;
    double targetAngle = angle % 360;
    double error = targetAngle - currentAngle;
    double speed = (Constants.DRIVETRAIN_ROTATION_kP * error);
    return reverseRotation ? speed : -speed;
  }

  public void turnToAngle(double angle) {
    var chassis = new ChassisSpeeds(0, 0, getRequiredTurningSpeedForAngle(angle));
    this.chassisSpeeds = chassis;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
}
