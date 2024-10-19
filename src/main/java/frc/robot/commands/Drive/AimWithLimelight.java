// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class AimWithLimelight extends Command {
  CommandSwerveDrivetrain drivetrain;
  Limelight mLimelight;
  PIDController LimelightTurnPID = new PIDController(1.5,0, 0);
  /** Creates a new AimWithLimelight. */
  public AimWithLimelight(CommandSwerveDrivetrain drivetrain, Limelight mLimelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.mLimelight = mLimelight;
    addRequirements(drivetrain, mLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandSwerveDrivetrain.setAutonomousShotHeading(Limelight.getRotationLime());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("In Command");
    System.out.println(Limelight.getRotationLime().getDegrees());
    //CommandSwerveDrivetrain.AimWithLimelight(()-> Limelight.getRotationLimelight(), drivetrain);
    CommandSwerveDrivetrain.drive.withRotationalRate(LimelightTurnPID.calculate(-Limelight.getRotationLime().getDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
      return false;
    
    
  }
}
