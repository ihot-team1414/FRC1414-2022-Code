package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public DriveCommand(
        DrivetrainSubsystem drivetrain,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSupplier
    ) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double translationXPercent = translationXSupplier.getAsDouble();
        double translationYPercent = translationYSupplier.getAsDouble();
        double rotationPercent = rotationSupplier.getAsDouble();

        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXPercent * Constants.DRIVETRAIN_MAX_VEL,
                        translationYPercent * Constants.DRIVETRAIN_MAX_VEL,
                        rotationPercent * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        drivetrain.getRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
