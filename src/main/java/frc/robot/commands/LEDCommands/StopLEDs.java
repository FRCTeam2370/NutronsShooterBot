// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class StopLEDs extends Command {
  LEDSubsystem mLedSubsystem = new LEDSubsystem();
  /** Creates a new StopLEDs. */
  public StopLEDs() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mLedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LEDSubsystem.setColor(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
