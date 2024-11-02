// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  public static TalonFX IntakeMotor = new TalonFX(11);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    resetIntake();
  }


  private static void resetIntake(){
    IntakeMotor.setInverted(true);
  }

  public static void runIntake(double speed){
    IntakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
