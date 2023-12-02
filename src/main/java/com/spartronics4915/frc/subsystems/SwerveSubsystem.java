package com.spartronics4915.frc.subsystems;

import java.io.File;
import java.io.IOException;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static com.spartronics4915.frc.Constants.Swerve.kMaxDriveSpeed;
import static com.spartronics4915.frc.Constants.Swerve.kMaxSpeedPossible;
import static com.spartronics4915.frc.Constants.Swerve.kMaxAngularSpeed;
import static com.spartronics4915.frc.Constants.Swerve.kDeadband;
import static com.spartronics4915.frc.Constants.Swerve.kResponseCurveExponent;

public final class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive mSwerveDrive;
    private CommandXboxController mDriverController;

    public SwerveSubsystem(CommandXboxController driverController) {
        mDriverController = driverController;
        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
        // In this case the gear ratio is 150/7 motor revolutions per wheel rotation.
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(150. / 7, 1);

        // Motor conversion factor is (PI * WHEEL DIAMETER) / (GEAR RATIO * ENCODER
        // RESOLUTION).
        // In this case the wheel diameter is 4 inches.
        // The gear ratio is 6.75 motor revolutions per wheel rotation.
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75, 1);

        // If you did not want to specify conversion factors in these JSON files you can
        // place them as parameters for createSwerveDrive.

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            mSwerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(kMaxSpeedPossible, angleConversionFactor, driveConversionFactor);
        } catch (IOException err) {
            err.printStackTrace(System.err);
            System.exit(-1);
        }
        System.out.println("Maximum speed: " + kMaxSpeedPossible);
        Command defaultCommand = Commands.run(this::BasicTeleop, this);
        setDefaultCommand(defaultCommand);
    }

    public void BasicTeleop() {
        ChassisSpeeds chassisSpeeds = mSwerveDrive.getSwerveController().getRawTargetSpeeds(
                Math.pow(MathUtil.applyDeadband(mDriverController.getLeftX(), kDeadband), kResponseCurveExponent)
                        * kMaxDriveSpeed,
                Math.pow(MathUtil.applyDeadband(mDriverController.getLeftY(), kDeadband), kResponseCurveExponent)
                        * kMaxDriveSpeed,
                Math.pow(MathUtil.applyDeadband(mDriverController.getRightX(), kDeadband), kResponseCurveExponent)
                        * kMaxAngularSpeed);

        mSwerveDrive.drive(chassisSpeeds, false, new Translation2d(0, 0));
        CANSparkMax motorController = (CANSparkMax) mSwerveDrive.getModules()[0].getDriveMotor().getMotor();
        SmartDashboard.putNumber("Motor 0 Set Speed", motorController.get());

    }

    public void AngleOnlyTeleop() {

        Translation2d stickReading = new Translation2d(mDriverController.getRightX(),
                mDriverController.getRightY());

        // Only drive if the stick is being pushed
        if (stickReading.getNorm() < 0.5)
            return;

        Rotation2d targetAngle = stickReading.getAngle();
        SwerveModuleState targetState = new SwerveModuleState(0, targetAngle);
        for (var m : mSwerveDrive.getModules()) {
            m.setDesiredState(targetState, false, true);
        }
        System.out.println("Asking for rotation " + targetAngle);
    }

    @Override
    public void periodic() {
        // AngleOnlyTeleop();
        //BasicTeleop();
        

    }

    public Pose2d getPose() {
        return mSwerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        mSwerveDrive.resetOdometry(initialHolonomicPose);
    }
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
      mSwerveDrive.setChassisSpeeds(chassisSpeeds);
    }
}
