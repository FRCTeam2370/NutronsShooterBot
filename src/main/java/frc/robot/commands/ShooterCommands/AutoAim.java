// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoAim extends Command {
  Shooter mShooter;
  Limelight mLimelight;
  /** Creates a new AutoAim. */
  public AutoAim(Limelight mLimelight, Shooter mShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooter, mLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.AimShooterWithDistance(Limelight.DistanceFromTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Shooter.shooterAim.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Shooter.aimSetPoint > Shooter.aimSetPoint*0.9){
      return true;
    }else{
      return false;
    }
  }
}
