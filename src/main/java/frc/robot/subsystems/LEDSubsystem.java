// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDSubsystem extends SubsystemBase {
  public static AddressableLED headlights = new AddressableLED(9);
  private static AddressableLEDBuffer headlightsBuf = new AddressableLEDBuffer(12);
  
  private static int green = 0;
  private static int applyer = 1;
  
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    startLEDs();
  }

  private static void startLEDs(){
    headlights.setLength(headlightsBuf.getLength());
    headlights.start();
  }

  public static void setColor(int r, int b, int g){
    for(int i = 0; i < headlightsBuf.getLength(); i++){
      headlightsBuf.setRGB(i, r, g, b);
    }

    headlights.setData(headlightsBuf);
  }

  public static void LEDsResting(){
    for(int i=0; i < headlightsBuf.getLength(); i++){
      headlightsBuf.setRGB(i, 50, 255, 0);
    }
  }

  public static void NotePulse(double speed){
    
    for(int i=0; i < headlightsBuf.getLength(); i++){
      headlightsBuf.setRGB(i, 0, 0, green);
    }

    green += speed *applyer;

    if(green == 255){
      applyer = -1;
    }else if(green == 0){
      applyer = 1;
    }
    
    headlights.setData(headlightsBuf);
  }

  public static void GreenPulse(double speed){
    
    for(int i=0; i < headlightsBuf.getLength(); i++){
      headlightsBuf.setRGB(i, 0, green, 0);
    }

    green += applyer;

    if(green == 100){
      applyer = -1;
    }else if(green == 0){
      applyer = 1;
    }
    
    headlights.setData(headlightsBuf);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
