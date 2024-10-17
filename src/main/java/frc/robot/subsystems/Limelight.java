// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  public static double tx;
  public static double DistanceFromTarget;
  public static double ty;
  /** Creates a new Limelight. */
  public Limelight() {
    LimelightHelpers.setPipelineIndex("limelight", 0);
  }

  public static void FindDistance(){
    double ty = LimelightHelpers.getTY("");
    double limelightMountHeight = Constants.limelightConstants.limelightHeight;
    double limelightMountPitch = Constants.limelightConstants.limelightPitch;

    double angleToGoalDegrees = limelightMountPitch + ty;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    double distanceFromTarget = (57.13 - limelightMountHeight) / Math.tan(angleToGoalRadians);
    double distanceFromTargetInches = distanceFromTarget * 0.503;
    SmartDashboard.putNumber("Li DistanceFromTarget", distanceFromTarget);
    //Shooter.AimShooterWithDistance(distanceFromTarget);
    DistanceFromTarget = distanceFromTarget;
    getRotationLime();
    
}

public static double txSlowly(){
  double TX = getTx() * 0.1;
  return TX;
}

public static Rotation2d getRotationLime(){
  double rotationalOffset = getTx() * -0.1;
  Rotation2d angle = (Rotation2d.fromDegrees(rotationalOffset));
  return angle;
}

public static double getRotationLimelight(){
  double angle = txSlowly();
  return angle;
}

  public static double getTx(){
    tx = LimelightHelpers.getTX("");
    return tx;
  }

  public static double getTy(){
    ty = LimelightHelpers.getTY("");
    return ty;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Limelight tx", tx);
    FindDistance();
  }
}
