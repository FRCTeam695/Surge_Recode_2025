package frc.robot.Subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// import edu.wpi.first.networktables.BooleanSubscriber;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDs extends SubsystemBase {

  private final AddressableLED m_LED;
  private final AddressableLEDBuffer m_LED_Buffer;
  // private final BooleanSubscriber amplifySub;
  // private final NetworkTable table;
  // private final NetworkTableInstance inst;
  private String color;

  public LEDs() {
    // LED
    m_LED = new AddressableLED(0);
    m_LED_Buffer = new AddressableLEDBuffer(27);
    m_LED.setLength(m_LED_Buffer.getLength());
    color = "";
    // NT
    // inst = NetworkTableInstance.getDefault();
    // table = inst.getTable("SideCar");
    // amplifySub = table.getBooleanTopic("Amplify").subscribe(false);

    m_LED.setData(m_LED_Buffer);
    m_LED.start();
    //color = "green";
  }

  public Command setColorToWhite() {
    if(color.equals("amplify")) return new WaitCommand(0);
    SmartDashboard.putString("Color", "white");
    return runOnce(()-> color = "white");
  }

  public Command setColorToOrange() {
    if(color.equals("amplify")) return new WaitCommand(0);
    SmartDashboard.putString("Color", "orange");
    return runOnce(()-> color = "orange");
  }

  public Command setColorToGreen() {
    if(color.equals("amplify")) return new WaitCommand(0);
    SmartDashboard.putString("Color", "green");
    return runOnce(()-> color = "green");
  }

  public Command turnColorOff() {
    if(color.equals("amplify")) return new WaitCommand(0);
    SmartDashboard.putString("Color", "off");
    return runOnce(()-> color = "off");
  }

  public Command amplifyLED() {
    SmartDashboard.putString("Color", "amp");
    return runOnce(()-> color = "amplify");
  }

  public Command deAmplify(){
    return runOnce(()-> color = "off");
  }

  public String getColor(){
    return color;
  }

  public void updateColor() {
    //OVERRIDE COLOR IF AMPLIFY COMMAND
     if (color.equals("amplify")) {
      for (int i = 0; i < m_LED_Buffer.getLength(); ++i) {
        m_LED_Buffer.setRGB(i, 255, 255, 255);
      }
      m_LED.setData(m_LED_Buffer);
      return;
    }
    switch (color) {
      case "off":
        for (int i = 0; i < m_LED_Buffer.getLength(); ++i) {
          m_LED_Buffer.setRGB(i, 255, 0, 255);
        }
        break;
      case "green":
        for (int i = 0; i < m_LED_Buffer.getLength(); ++i) {
          m_LED_Buffer.setRGB(i, 0, 255, 0);
        }
        break;
      case "orange":
        for (int i = 0; i < m_LED_Buffer.getLength(); ++i) {
          m_LED_Buffer.setRGB(i, 255, 20, 0);
        }
        break;
      default:
        for (int i = 0; i < m_LED_Buffer.getLength(); ++i) {
          m_LED_Buffer.setRGB(i, 255, 0, 255);
        }
        break;
    }
    m_LED.setData(m_LED_Buffer);

  }

  @Override
  public void periodic() {
    updateColor();
  }
}