// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5461; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.55245; // FIXME Measure and set wheelbase

    // public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3; // FIXME Set front left steer encoder ID
    public static final Rotation2d FRONT_LEFT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(0); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6; // FIXME Set front right steer encoder ID
    public static final Rotation2d FRONT_RIGHT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(0); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 21; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 22; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 20; // FIXME Set back left steer encoder ID
    public static final Rotation2d BACK_LEFT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(0); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; // FIXME Set back right steer encoder ID
    public static final Rotation2d BACK_RIGHT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(0); // FIXME Measure and set back right steer offset
    
    
    public static final int ROLLING_AVERAGE_SIZE = 5;
    public static final double TIME_STEP = 0.1;


    public static final double LIMELIGHT_HEIGHT = 0; // TODO
    public static final double LIMELIGHT_Y_ANGLE = 0;

    public static final double TARGET_HEIGHT = 2.64;
    public static final double FLYWHEEL_RADIUS = 0.05;

    public static final int SHOOTER_ID_1 = 16; // TODO
    public static final int SHOOTER_ID_2 = 17; // TODO
    public static final int HOOD_SERVO_ID_1 = 0; // TODO
    public static final int HOOD_SERVO_ID_2 = 0; // TODO

    public static final int TURRET_MOTOR_ID = 15; // TODO
    public static final double TURRET_MOTOR_MAX_OUTPUT = 1; // TODO

    public static final double TURRET_MOTOR_kF = 0; // TODO
    public static final double TURRET_MOTOR_kI = 0; // TODO
    public static final double TURRET_MOTOR_kP = 0.1; // TODO
    public static final double TURRET_MOTOR_kD = 0; // TODO

    
    public static final int LOADER_FRONT_MOTOR_ID = 9; // 
    public static final int LOADER_BACK_MOTOR_ID = 23; // 
    public static final int FUNNEL_MOTOR_ID = 5; // TODO


    public static final int CLIMB_TELESCOPING_ARM_1_MOTOR_ID = 18;
    public static final int CLIMB_TELESCOPING_ARM_2_MOTOR_ID = 14;

    public static final int CLIMB_ARM_1_MOTOR_ID = 13;
    public static final int CLIMB_ARM_2_MOTOR_ID = 19;


    public static final double CLIMB_ARM_ALLOWED_ERROR = 0;
    public static final double PIVOT_ARM_MOTOR_MAX_OUTPUT = 0;
    public static final double PIVOT_ARM_MOTOR_kF = 0;
    public static final double PIVOT_ARM_MOTOR_kP = 0;
    public static final double PIVOT_ARM_MOTOR_kI = 0;
    public static final double PIVOT_ARM_MOTOR_kD = 0;


    public static final double TELESCOPING_ARM_MOTOR_MAX_OUTPUT = 0;
    public static final double TELESCOPING_ARM_MOTOR_kF = 0;
    public static final double TELESCOPING_ARM_MOTOR_kP = 0;
    public static final double TELESCOPING_ARM_MOTOR_kI = 0;
    public static final double TELESCOPING_ARM_MOTOR_kD = 0;
    public static final double CLIMB_ELEVATOR_ALLOWED_ERROR = 0;


    public static final int INTAKE_ID = 4;


    public static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.REVPH;
    public static final int FRONT_INTAKE_FORWARD_ID = 0;
    public static final int FRONT_INTAKE_RETRACTED_ID = 0;

    public static final Value INTAKE_PISTON_OPEN = Value.kReverse;
    public static final Value INTAKE_PISTON_CLOSED = Value.kForward;  

    

}
