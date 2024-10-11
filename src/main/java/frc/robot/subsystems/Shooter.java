// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  public static TalonFX shooter1, shooter2, shooterAim, shooterIndex;

  private static TalonFXConfiguration AimMotorConfig = new TalonFXConfiguration();
  private static PositionDutyCycle aimDutyCycle = new PositionDutyCycle(0);

  private static TalonFXConfiguration IndexConfig = new TalonFXConfiguration();
  private static TalonFXConfiguration shooterMotorConfiguration = new TalonFXConfiguration();

  private static PositionDutyCycle indexDutyCycle = new PositionDutyCycle(0);
  private static VelocityDutyCycle shooterVelocityDC = new VelocityDutyCycle(0);

  private static SlewRateLimiter ShooterLimiter = new SlewRateLimiter(5);
  private static SlewRateLimiter IndexLimiter = new SlewRateLimiter(3.5);

  private static double shooterAimkp;
  private static double shooterAimkI;
  private static double shooterAimkD;

  public static final DigitalInput ShooterIndexerBeam = new DigitalInput(0);

  public static Boolean hasPiece;

  public static double posSliderVal;

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

    SmartDashboard.putNumber("pos Slider", posSliderVal);
  }

  public static void runShooter(double speed){
    if(speed <= -10 || speed >= 5){
      shooter1.setControl(shooterVelocityDC.withVelocity(speed));
      shooter2.setControl(shooterVelocityDC.withVelocity(-speed));
    }else{
      shooter1.set(0);
      shooter2.set(0);
    }
    
  }

  public static void AimShooter(double position){
    shooterAim.setControl(aimDutyCycle.withPosition(position));
  }

  public static void runIndex(double speed){
    if(speed > 0.05 || speed < -0.05){
      shooterIndex.set(speed);
    }else{
      shooterIndex.set(0);
      shooterIndex.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  public static boolean getShooterIndexerBeam(){
    boolean val;
    val = ShooterIndexerBeam.get();
    return val;
  }

  private static void configAimMotor(){
    AimMotorConfig.Slot0.kP = 0.15;//0.025  0.125
    AimMotorConfig.Slot0.kI = 0;
    AimMotorConfig.Slot0.kD = 0; 
    AimMotorConfig.Slot0.kG = 0.03;//0.45

    shooterAim.getConfigurator().apply(AimMotorConfig);
    shooterAim.setInverted(true);

    shooterAim.setPosition(0);
    shooterAim.setNeutralMode(NeutralModeValue.Brake);
  }

  private static void configIndexMotor(){
    shooterIndex.setNeutralMode(NeutralModeValue.Brake);

    IndexConfig.Slot0.kP = 0.035;
    IndexConfig.Slot0.kI = 0;
    IndexConfig.Slot0.kD = 0;

    IndexConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.3;

    shooterIndex.getConfigurator().apply(IndexConfig);

    shooterIndex.setPosition(0);
  }

  private static void configShooterMotors(){
    shooter2.setInverted(true);
    shooter1.setInverted(false);

    shooterMotorConfiguration.Slot0.kV = 0.011;
    shooterMotorConfiguration.Slot0.kP = 0.01;
    shooter1.getConfigurator().apply(shooterMotorConfiguration);
    shooter2.getConfigurator().apply(shooterMotorConfiguration);
  }

  public static double getShooterVelocity(){
    double v = (shooter1.getVelocity().getValueAsDouble() + -shooter2.getVelocity().getValueAsDouble()) /2;
    return v;
  }

  public static void indexABit(double pos){
    shooterIndex.setControl(indexDutyCycle.withPosition(shooterIndex.getPosition().getValueAsDouble() + pos));
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

    SmartDashboard.putBoolean("Shooter Indexer Beam Break", getShooterIndexerBeam());

    SmartDashboard.putNumber("ShooterVelocity", getShooterVelocity());

    if(getShooterIndexerBeam() == false){
      hasPiece = true;
    } else {
      hasPiece = false;
    }

    SmartDashboard.putBoolean("Has Piece", hasPiece);

    SmartDashboard.putNumber("Shooter Aim Pos", shooterAim.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Aim Error", shooterAim.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber("Aim Motor Volts", shooterAim.getMotorVoltage().getValueAsDouble());

    posSliderVal = SmartDashboard.getNumber("pos Slider", 0.5);
    

  }
}
