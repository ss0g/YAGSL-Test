package com.spartronics4915.frc.commands.auto;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.spartronics4915.frc.subsystems.SwerveSubsystem;

public final class GenerateAutoCommands {

    public static Command followSquarePath(SwerveSubsystem swerve) {
        List<PathPlannerTrajectory> loadedPath = PathPlanner.loadPathGroup("SquarePath", new PathConstraints(4, 3));
        // This is just an example event map. It would be better to have a constant,
        // global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerve::getPose,
                swerve::resetOdometry,
                new PIDConstants(0.7, 0, 0), 
                new PIDConstants(0.4, 0, 0),
                swerve::setChassisSpeeds, eventMap, swerve);

        
        return Commands.sequence(autoBuilder.fullAuto(loadedPath));
    }

}
