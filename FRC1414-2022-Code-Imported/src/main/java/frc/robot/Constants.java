package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class Constants {
    public static final Pose2d STARTING_POSITIONS[] = {new Pose2d(8, 2.84, Rotation2d.fromDegrees(68)), new Pose2d(0, 0, Rotation2d.fromDegrees(0))};
    
    // DRIVETRAIN
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5969;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5969;
    public static final double DRIVETRAIN_MAX_VOLTAGE = 8;
    public static final double DRIVETRAIN_MAX_VEL = 5;
    public static final double DRIVETRAIN_MAX_ACCELERATION = 3;

    public static final double DRIVETRAIN_PATH_X_kP = 1.5;
    public static final double DRIVETRAIN_PATH_Y_kP = 1.5;
    public static final double DRIVETRAIN_PATH_THETA_kP = 3;

    public static final double DRIVETRAIN_ROTATION_kP = 0.0875;

    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
      Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    public static TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(
        Constants.DRIVETRAIN_MAX_VEL,
        Constants.DRIVETRAIN_MAX_ACCELERATION)
        .setKinematics(Constants.KINEMATICS);

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.DRIVETRAIN_MAX_VEL /
    Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(87 + 180);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(23);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 22;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 21;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 20;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(129 + 180);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(303);
    
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

    public static final double SHOOTER_kF = 0.0544833;
    public static final double SHOOTER_kP =  0.01051414;
    public static final double SHOOTER_kI = 0.0;
    public static final double SHOOTER_kD = 0.0;
    public static final double SHOOTER_MAX_OUTPUT = 0.75;

    public static final int TURRET_MOTOR_ID = 15;
    public static final double TURRET_MOTOR_MAX_OUTPUT = 0.65;
    public static final double TURRET_MAX_POS = 5500;
    public static final double TURRET_MIN_POS = -5500;

    public static final double TURRET_MOTOR_kP = 50;
    public static final double TURRET_ARM_MOTOR_kF = 0;
    public static final double TURRET_ARM_MOTOR_kP = 0.5;
    public static final double TURRET_ARM_MOTOR_kI = 0;
    public static final double TURRET_ARM_MOTOR_kD = 0;
    public static final double TURRET_ARM_MAX_VEL = 3000;
    public static final double TURRET_ARM_ACCEL = 3000;

    public static final int LOADER_FRONT_MOTOR_ID = 9;
    public static final int LOADER_BACK_MOTOR_ID = 23;
    public static final int FUNNEL_MOTOR_ID = 5;
    public static final int INTAKE_ID = 4;

    public static final double LOADING_SPEED = 0.85;
    public static final double HOLDING_SPEED = 0.75;
    public static final double FUNNEL_SPEED = 0.25;
    public static final double INTAKE_SPEED = 0.75;

    public static final int TELESCOPING_ARM_1_MOTOR_ID = 14;
    public static final int TELESCOPING_ARM_2_MOTOR_ID = 18;
    public static final double TELESCOPING_ARM_ALLOWED_ERROR = 1000;
    public static final double TELESCOPING_ARM_MOTOR_MAX_OUTPUT = 1;
    public static final double TELESCOPING_ARM_MOTOR_kF = 0;
    public static final double TELESCOPING_ARM_MOTOR_kP = 0.03;
    public static final double TELESCOPING_ARM_MOTOR_kI = 0;
    public static final double TELESCOPING_ARM_MOTOR_kD = 0;
    public static final double TELESCOPING_ARM_MAX_VEL = 10000;
    public static final double TELESCOPING_ARM_ACCEL = 3000;

    public static final int PIVOT_ARM_1_MOTOR_ID = 13;
    public static final int PIVOT_ARM_2_MOTOR_ID = 19;
    public static final double PIVOT_ARM_ALLOWED_ERROR = 0;
    public static final double PIVOT_ARM_MOTOR_MAX_OUTPUT = 1;
    public static final double PIVOT_ARM_MOTOR_kF = 0;
    public static final double PIVOT_ARM_MOTOR_kP = 0.018;
    public static final double PIVOT_ARM_MOTOR_kI = 0;
    public static final double PIVOT_ARM_MOTOR_kD = 0;
    public static final double PIVOT_ARM_MAX_VEL = 20000;
    public static final double PIVOT_ARM_ACCEL = 5000;

    public static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.REVPH;

    public static final Value INTAKE_PISTON_OPEN = Value.kForward;
    public static final Value INTAKE_PISTON_CLOSED = Value.kReverse;
    public static final int INTAKE_PISTON_FORWARD = 8;
    public static final int INTAKE_PISTON_REVERSE = 13;
    
}
