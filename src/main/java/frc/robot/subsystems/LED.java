// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */


   AddressableLED m_led0 = new AddressableLED(0);



  private AddressableLEDBuffer m_ledBuffer0 = new AddressableLEDBuffer(560);

  private int i = 4;
  private int flashtimer = 0;
  public int time = 0;
  public boolean isEnabled = false;
  public boolean intakeNote = false;
  public boolean shootReady = false;
  public boolean climbTime = false;

  public boolean flashDone = false;
  private int flashcount = 0;
  private int m_rainbowFirstPixelHue;

  public LED() {
    m_led0.setLength(m_ledBuffer0.getLength());
    m_led0.start();
   }

  @Override
  public void periodic() {
    
    if (isEnabled == false) {

      if(DriverStation.getAlliance().isPresent()){
        if(DriverStation.getAlliance().get() == Alliance.Blue){
          m_ledBuffer0.setRGB(i, 0, 0, 255);
        } else{
          m_ledBuffer0.setRGB(i, 255, 0, 0);
        }
      }
      
      i++;
      if(i > (m_ledBuffer0.getLength()-1)){
        i = 5;
      }

      m_ledBuffer0.setRGB(i-4,0,0,0);
      m_led0.setData(m_ledBuffer0);

    } else{
      if(climbTime){
        rainbow();
        m_led0.setData(m_ledBuffer0);
      } else if(intakeNote && !flashDone){
        if(flashcount%2 == 1){
          for (var i = 0; i < m_ledBuffer0.getLength()-1; i = i + 1) {
              m_ledBuffer0.setRGB(i, 150,150,150);
          }
        } else{
            for (var i = 0; i < m_ledBuffer0.getLength()-1; i = i + 1) {
              m_ledBuffer0.setRGB(i, 0,0,0);
          }
        }
          flashtimer++;
          if(flashtimer > 10){
            flashtimer = 0;
            flashcount++;
          }
          if(flashcount == 5){
            flashcount = 0;
            flashtimer = 0;
            flashDone = true;
          }
          m_led0.setData(m_ledBuffer0);
      } else if(shootReady && !flashDone){
          for (var i = 0; i < m_ledBuffer0.getLength()-1; i = i + 1) {
              m_ledBuffer0.setRGB(i, 130,130,130);
          }
          flashtimer++;
          if(flashtimer > 10){
            flashtimer = 0;
            flashcount++;
          }
          if(flashcount == 5){
            flashDone = true;
          }
          m_led0.setData(m_ledBuffer0);
      } else {
        if(DriverStation.getAlliance().isPresent()){
          if(DriverStation.getAlliance().get() == Alliance.Blue){

rainbow();
            // for (var i = 0; i < m_ledBuffer0.getLength()-1; i = i + 1) {
            //   m_ledBuffer0.setRGB(i, 0, 0, 200);
            // }
          }else{

            
            if(DriverStation.getAlliance().get() == Alliance.Red){
              // for (var i = 0; i < m_ledBuffer0.getLength()-1; i = i + 1) {
              //   m_ledBuffer0.setRGB(i, 200, 0, 0);
              // }

              rainbow();
              
            }
          }
        }
        m_led0.setData(m_ledBuffer0);
      }
    }
  }


  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer0.getLength()-1; i = i + 1) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer0.getLength())) % 180;
      // Set the value
      m_ledBuffer0.setHSV(i, hue, 255, 128);

    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 2;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

}