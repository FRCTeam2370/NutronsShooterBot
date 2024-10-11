// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  Shooter mShooter = new Shooter();
  IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  double speed;
  double IndexSpeed;
  /** Creates a new Shoot. */
  public Shoot(double speed, Shooter mShooter, IntakeSubsystem mIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    this.mShooter = mShooter;
    this.mIntakeSubsystem = mIntakeSubsystem;
    addRequirements(mShooter, mIntakeSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IndexSpeed = (speed > 60) ? (speed/2) : 0.3;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.runShooter(speed);
    if(Shooter.getShooterVelocity() > (speed * 0.9) && Shooter.hasPiece && Shooter.shooterAim.getPosition().getValueAsDouble() > 0.75){
      Shooter.runIndex(IndexSpeed);
      IntakeSubsystem.runIntake(IndexSpeed);
    }else{
      Shooter.runIndex(0);
      IntakeSubsystem.runIntake(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.runShooter(0);
    Shooter.runIndex(0);
    IntakeSubsystem.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Shooter.hasPiece = false){
      return true;
    }else{
      return false;
    }
    
  }
}
