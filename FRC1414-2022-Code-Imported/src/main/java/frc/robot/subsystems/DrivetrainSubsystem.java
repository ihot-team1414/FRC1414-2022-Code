package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import frc.util.Realsense;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;

public class DrivetrainSubsystem extends SubsystemBase {
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.DRIVETRAIN_MAX_VEL /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);



    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final Pose2d startingPosition;

    private final AHRS gyroscope = new AHRS(I2C.Port.kMXP);

    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
    
    private final SwerveDriveOdometry odometry;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);


    public DrivetrainSubsystem(Pose2d startingPosition) {
        Realsense.getInstance().setInitial(startingPosition.getX(), startingPosition.getY(), startingPosition.getRotation().getDegrees());
        this.startingPosition = startingPosition;
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        frontLeftModule = Mk4SwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
        Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                Constants.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk4SwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk4SwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                Constants.BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk4SwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                Constants.BACK_RIGHT_MODULE_STEER_OFFSET
        );

        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(-gyroscope.getFusedHeading()), startingPosition);

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
    }

    public void zeroGyroscope() {
        odometry.resetPosition(
                new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
                Rotation2d.fromDegrees(-gyroscope.getFusedHeading())
        );
    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public double getGyroAngle() {
      return Rotation2d.fromDegrees(-gyroscope.getFusedHeading()).getRadians();
    }
    
    double kp = 0.0875;//0.03
    double ki = 0.0; //0.01
    double kd = 0.0;
    double lastHeadingError = 0.0;
    double errorAccumulated = 0.0;
  
    double ykp = 0.375;//0.03
    double yki = 0.0; //0.01
    double ykd = 0.0;
    double lastYError = 0.0;
    double yErrorAccumulated = 0.0;
  
    double xkp = 0.375;//0.03
    double xki = 0.0; //0.01
    double xkd = 0.0;
    double lastXError = 0.0;
    double xErrorAccumulated = 0.0;
  
    public double getRequiredTurningSpeedForAngle(double angle) {
      double currentAngle = this.getRotation().getDegrees() % 360;
      double error = angle - currentAngle;
      this.errorAccumulated += error * Constants.TIME_STEP;
      double speed = (kp * error);
      this.lastHeadingError = error;
      return -speed;
    }

    public void turnToAngle(double angle) {
        var chassis = new ChassisSpeeds(0, 0, getRequiredTurningSpeedForAngle(angle));
        this.chassisSpeeds = chassis;
    }
  
    public Pose2d getPose() {
      return odometry.getPoseMeters();
    }

    public void resetErrors() {
      this.lastHeadingError = 0.0;
      this.errorAccumulated = 0.0;
    }

    public void setModuleStates(SwerveModuleState[] states) {
      frontLeftModule.set(states[0].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[0].angle.getRadians());
      frontRightModule.set(states[1].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[1].angle.getRadians());
      backLeftModule.set(states[2].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[2].angle.getRadians());
      backRightModule.set(states[3].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[3].angle.getRadians());
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(-gyroscope.getFusedHeading()),
                new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()))
        );

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        frontLeftModule.set(states[0].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / Constants.DRIVETRAIN_MAX_VEL * Constants.DRIVETRAIN_MAX_VOLTAGE, states[3].angle.getRadians());
    }
}
