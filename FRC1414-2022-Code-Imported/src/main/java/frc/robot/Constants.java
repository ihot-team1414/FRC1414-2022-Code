package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.util.Color;


/*
 * In PercentVbus, the output is between -1.0 and 1.0, with 0.0 as stopped.
   * In Follower mode, the output is the integer device ID of the talon to
   * duplicate.
   * In Voltage mode, outputValue is in volts.
   * In Current mode, outputValue is in amperes.
   * In Speed mode, outputValue is in position change / 10ms.
   * In Position mode, outputValue is in encoder ticks or an analog
   * value, depending on the sensor.
   *
 */

public final class Constants {
  public static final boolean MANUAL_SPEED_AND_ANGLE = false;

  public static final Pose2d[] STARTING_POSITIONS = {
    new Pose2d(8, 2.84, Rotation2d.fromDegrees(-108)),
    new Pose2d(6.04, 4.7, Rotation2d.fromDegrees(0))
  };

  public static final double FALCON_500_STALL_CURRENT = 250;

  // DRIVETRAIN
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5969;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5969;
  public static final double DRIVETRAIN_MAX_VOLTAGE = 8;
  public static final double DRIVETRAIN_MAX_VEL = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
          SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI * 2;
  public static final double DRIVETRAIN_MAX_ACCELERATION = 2;


  public static final double DRIVETRAIN_PATH_X_kP = 52e-1;
  public static final double DRIVETRAIN_PATH_X_kI = 0.0;
  public static final double DRIVETRAIN_PATH_X_kD = 0.0;
  
  public static final double DRIVETRAIN_PATH_Y_kP = 52e-1;
  public static final double DRIVETRAIN_PATH_Y_kI = 0.0;
  public static final double DRIVETRAIN_PATH_Y_kD = 0.0;

  public static final double DRIVETRAIN_PATH_THETA_kP = 6e-1;
  public static final double DRIVETRAIN_PATH_THETA_kI = 0.0;
  public static final double DRIVETRAIN_PATH_THETA_kD = 15e-3;

  public static final double DRIVETRAIN_ROTATION_kP = 0.0875;

  public static final double DRIVETRAIN_VISION_ROTATION_kP = 0.000875;

  public static final Constraints THETA_CONTROLLER_CONSTRAINTS = new Constraints(360, 180);

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * SdsModuleConfigurations.MK3_FAST.getDriveReduction() * SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 
    Constants.MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(
    Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0
  );
  
  public static TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(
    Constants.DRIVETRAIN_MAX_VEL * 0.75,
    Constants.DRIVETRAIN_MAX_ACCELERATION
  );

  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(
      Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0
    ),
    new Translation2d(
      Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0
    ),
    new Translation2d(
      -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0
    ),
    new Translation2d(
      -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0
    )
  );

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(36);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 8;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(52 + 180);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 22;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 21;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 20;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(192);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(126 + 180);

  public static final int ROLLING_AVERAGE_SIZE = 5;
  public static final double TIME_STEP = 0.1;

  public static final double LIMELIGHT_HEIGHT = 0.66;
  public static final double LIMELIGHT_Y_ANGLE = 35;

  public static final double TARGET_HEIGHT = 2.64;
  public static final double FLYWHEEL_RADIUS = 0.05;

  public static final int SHOOTER_ID_1 = 16;
  public static final int SHOOTER_ID_2 = 17;
  public static final int HOOD_SERVO_ID_1 = 9;
  public static final int HOOD_SERVO_ID_2 = 8;
  public static final double HOOD_MIN = 0.24;
  public static final double HOOD_MAX = 0.5;

  public static final double SHOOTER_kF = 0.05;
  public static final double SHOOTER_kP = 0.075;
  public static final double SHOOTER_kI = 0.0;
  public static final double SHOOTER_kD = 0.0;
  public static final double SHOOTER_MAX_OUTPUT = 0.75;
  public static final double SHOOTER_ALLOWED_ERROR = 200;
  public static final double SHOOTER_LAYUP_SPEED = 3750;
  public static final double SHOOTER_MIN_LOAD_SPEED = 3000;

  public static final int TURRET_MOTOR_ID = 15;
  public static final double TURRET_MOTOR_MAX_OUTPUT = 0.6;
  public static final double TURRET_MAX_POS = 9500;
  public static final double TURRET_MIN_POS = -9500;
  public static final double TURRET_POSITION_ALLOWED_ERROR = 200;
  public static final double TURRET_MOTOR_POSITION_kF = 0.0;
  public static final double TURRET_MOTOR_POSITION_kP = 0.5;
  public static final double TURRET_MOTOR_POSITION_kI = 0;
  public static final double TURRET_MOTOR_POSITION_kD = 0;

  public static final double TURRET_VISION_ALLOWED_ERROR = 5;
  public static final double TURRET_MOTOR_VISTION_kP = 0.02;
  public static final double TURRET_MOTOR_VISTION_kI = 0.00;
  public static final double TURRET_MOTOR_VISTION_kD = 0.0;

  public static final int LOADER_FRONT_MOTOR_ID = 9;
  public static final int LOADER_BACK_MOTOR_ID = 23;
  public static final int FUNNEL_MOTOR_ID = 5;
  public static final int INTAKE_ID = 4;

  public static final double LOADING_SPEED = 0.75;
  public static final double HOLDING_SPEED = 0.75;
  public static final double FUNNEL_SPEED = 0.25;
  public static final double OUTTAKE_FUNNEL_SPEED = 0.1;
  public static final double INTAKE_SPEED = 0.75;

  public static final Color BLUE_TARGET = new Color(0.143, 0.427, 0.429);
  public static final Color RED_TARGET = new Color(0.561, 0.232, 0.114);

  public static final int TELESCOPING_ARM_1_MOTOR_ID = 14;
  public static final int TELESCOPING_ARM_2_MOTOR_ID = 18;
  public static final double TELESCOPING_ARM_ALLOWED_ERROR = 1000;
  public static final double TELESCOPING_ARM_MOTOR_MAX_OUTPUT = 1;
  public static final double TELESCOPING_ARM_MOTOR_kF = 0;
  public static final double TELESCOPING_ARM_MOTOR_kP = 0.03;
  public static final double TELESCOPING_ARM_MOTOR_kI = 0;
  public static final double TELESCOPING_ARM_MOTOR_kD = 0;
  public static final double TELESCOPING_ARM_MAX_VEL = 15000;
  public static final double TELESCOPING_ARM_ACCEL = 2000;
  public static final double TELESCOPING_ARM_SPOOL_SPEED = 0.1;

  public static final int PIVOT_ARM_1_MOTOR_ID = 13;
  public static final int PIVOT_ARM_2_MOTOR_ID = 19;
  public static final double PIVOT_ARM_ALLOWED_ERROR_FOR_TURRET = 8000;
  public static final double PIVOT_ARM_MOTOR_MAX_OUTPUT = 1;
  public static final double PIVOT_ARM_MOTOR_kF = 0;
  public static final double PIVOT_ARM_MOTOR_kP = 0.018;
  public static final double PIVOT_ARM_MOTOR_kI = 0;
  public static final double PIVOT_ARM_MOTOR_kD = 0;
  public static final double PIVOT_ARM_MAX_VEL = 90000;
  public static final double PIVOT_ARM_ACCEL = 90000;

  public static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;

  public static final Value INTAKE_PISTON_OPEN = Value.kForward;
  public static final Value INTAKE_PISTON_CLOSED = Value.kReverse;
  public static final int INTAKE_PISTON_FORWARD = 0;
  public static final int INTAKE_PISTON_REVERSE = 1;
}
