// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoAutoAim extends Command {
   Shooter mShooter;
  Limelight mLimelight;
  /** Creates a new AutoAim. */
  public AutoAutoAim(Limelight mLimelight, Shooter mShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mLimelight = mLimelight;
    this.mShooter = mShooter;
    addRequirements(mShooter, mLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.AimShooterWithDistance(Limelight.DistanceFromTarget);

    if(Shooter.shooterAim.getPosition().getValueAsDouble() > (Shooter.aimSetPoint * 0.95) && Shooter.hasPiece){
      Shooter.runShooter(85);
      if(Shooter.getShooterVelocity() > (85 * 0.9)){
        Shooter.runIndex(0.5);
      }
    }else{
      Shooter.runShooter(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.shooterAim.set(0);
    Shooter.runShooter(0);
    Shooter.runIndex(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Shooter.hasPiece == false){
      return true;
    }else{
      return false;
    }
  }
}
