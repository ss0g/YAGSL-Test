// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.spartronics4915.frc.commands.auto.GenerateAutoCommands;
import com.spartronics4915.frc.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController mDriverController = new CommandXboxController(0);

  private final SwerveSubsystem mSwerveSubsystem = new SwerveSubsystem(mDriverController);
  private final Command autoCommand;
  public RobotContainer() {
    configureBindings();

    autoCommand = GenerateAutoCommands.followSquarePath(mSwerveSubsystem);

  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoCommand;
  }

  public CommandXboxController getDriverController() {
    return mDriverController;
  }
}
