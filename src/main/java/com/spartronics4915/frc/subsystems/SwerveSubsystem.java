package com.spartronics4915.frc.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static com.spartronics4915.frc.Constants.Swerve.kMaxSpeed;
import static com.spartronics4915.frc.Constants.Swerve.kSpeedLimit;
import static com.spartronics4915.frc.Constants.Swerve.kMaxAngularSpeed;
import static com.spartronics4915.frc.Constants.Swerve.kDeadband;
import static com.spartronics4915.frc.Constants.Swerve.kResponseCurveExponent;
import static com.spartronics4915.frc.Constants.Swerve.kDriveMotorConversionFactor;
import static com.spartronics4915.frc.Constants.Swerve.kAngleMotorConversionFactor;

public final class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive mSwerveDrive;
    private CommandXboxController mDriverController;

    public SwerveSubsystem(CommandXboxController driverController) {
        mDriverController = driverController;
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            mSwerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(
                kMaxSpeed, kAngleMotorConversionFactor, kDriveMotorConversionFactor
            );
        } catch (IOException err) {
            err.printStackTrace(System.err);
            System.exit(-1);
        }
    }

    @Override
    public void periodic() {
        ChassisSpeeds chassisSpeeds = mSwerveDrive.getSwerveController().getRawTargetSpeeds(
            Math.pow(MathUtil.applyDeadband(mDriverController.getLeftX(), kDeadband), kResponseCurveExponent) * kSpeedLimit,
            Math.pow(MathUtil.applyDeadband(mDriverController.getLeftY(), kDeadband), kResponseCurveExponent) * kSpeedLimit,
            Math.pow(MathUtil.applyDeadband(mDriverController.getRightX(), kDeadband), kResponseCurveExponent) * kMaxAngularSpeed
        );
        mSwerveDrive.drive(chassisSpeeds, false, new Translation2d(0, 0));
    }
}
