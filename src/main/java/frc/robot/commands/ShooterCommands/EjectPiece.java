// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class EjectPiece extends Command {
  Shooter mShooter;
  IntakeSubsystem mIntakeSubsystem;
  /** Creates a new EjectPiece. */
  public EjectPiece(Shooter mShooter, IntakeSubsystem mIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooter, mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.runIndex(-0.10);
    IntakeSubsystem.runIntake(-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.runIndex(0);
    IntakeSubsystem.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
