// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  public static TalonFX shooter1, shooter2, shooterAim, shooterIndex;

  private static TalonFXConfiguration AimMotorConfig = new TalonFXConfiguration();
  private static PositionDutyCycle aimDutyCycle = new PositionDutyCycle(0);

  private static TalonFXConfiguration IndexConfig = new TalonFXConfiguration();
  private static PositionDutyCycle indexDutyCycle = new PositionDutyCycle(0);

  private static SlewRateLimiter ShooterLimiter = new SlewRateLimiter(5);

  private static double shooterAimkp;
  private static double shooterAimkI;
  private static double shooterAimkD;

  /** Creates a new Shooter. */
  public Shooter() {
    shooter1 = new TalonFX(20);
    shooter2 = new TalonFX(21);
    shooterIndex = new TalonFX(22);
    shooterAim = new TalonFX(23);

    configAimMotor();
    configIndexMotor();
    configShooterMotors();
    SmartDashboard.putNumber("Shooter Aim kP slider", 0);
    SmartDashboard.putNumber("Shooter Aim kI slider", 0);
    SmartDashboard.putNumber("Shooter Aim kD slider", 0);
  }

  public static void runShooter(double speed){
    shooter1.set(speed);
    shooter2.set(speed);
  }

  public static void AimShooter(double position){
    shooterAim.setControl(aimDutyCycle.withPosition(position));
  }

  public static void runIndex(double speed){
    shooterIndex.set(speed);
  }


  private static void configAimMotor(){
    shooterAim.setNeutralMode(NeutralModeValue.Brake);

    AimMotorConfig.Slot0.kP = shooterAimkp;//0.025
    AimMotorConfig.Slot0.kI = shooterAimkI;
    AimMotorConfig.Slot0.kD = shooterAimkD; 

    shooterAim.getConfigurator().apply(AimMotorConfig);

    shooterAim.setPosition(0);
  }

  private static void configIndexMotor(){
    shooterIndex.setNeutralMode(NeutralModeValue.Coast);

    IndexConfig.Slot0.kP = 0.035;
    IndexConfig.Slot0.kI = 0;
    IndexConfig.Slot0.kD = 0;

    shooterIndex.getConfigurator().apply(IndexConfig);

    shooterIndex.setPosition(0);
  }

  private static void configShooterMotors(){
    shooter1.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterAimkp = SmartDashboard.getNumber("Shooter Aim kP slider", 0);
    shooterAimkI = SmartDashboard.getNumber("Shooter Aim kI slider", 0);
    shooterAimkD = SmartDashboard.getNumber("Shooter Aim kD slider", 0);

    SmartDashboard.putNumber("Shooter Aim kP", shooterAimkp);
    SmartDashboard.putNumber("Shooter Aim kI", shooterAimkI);
    SmartDashboard.putNumber("Shooter Aim kD", shooterAimkD);
  }
}
