// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import team3176.robot.subsystems.RobotStateIO; 
import team3176.robot.subsystems.RobotStateIO.RobotStateIOInputs; 

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import team3176.robot.constants.RobotConstants.Status;
import team3176.robot.constants.SignalingConstants;

import team3176.robot.subsystems.superstructure.Claw;


public class RobotState extends SubsystemBase {

  private final RobotStateIO io;
  private final RobotStateIOInputs inputs = new RobotStateIOInputs();
  private static RobotState instance;
  private int wantedLEDState;
  private Claw m_Claw;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int i = 0;

  private double m_flashcounter = 0;
  private boolean leftfrontlowflash = false;
  private boolean leftfronthighflash = false;
  private boolean crosshighflash = false;
  private boolean rightfronthighflash = false;
  private boolean rightfrontlowflash = false;
  private boolean leftbackflash = false;
  private boolean crosslowflash = false;
  private boolean rightbackflash = false;
  private Color leftfrontlowcolor = Color.kBlack;
  private Color leftfronthighcolor = Color.kBlack;
  private Color crosshighcolor = Color.kBlack;
  private Color rightfronthighcolor = Color.kBlack;
  private Color rightfrontlowcolor = Color.kBlack;
  private Color leftbackcolor = Color.kBlack;
  private Color crosslowcolor = Color.kBlack;
  private Color rightbackcolor = Color.kBlack;
  private Color allcolor = Color.kBlack;
  private boolean allflash = false;

  private boolean isSolid;
  private boolean isFlashing;

  private Alliance alliance; 

  enum e_ClawPositionState {
    OPEN,
    CLOSED,
    IDLE
  }

  enum e_ClawRollersState {
    PosSPIN,  //Means front-most side of rollers spinning INTO the Claw, toward it's center 
    NegSPIN,  //Means front-most side of rollers spinning OUT-OF the Claw, toward it's exterior
    NoSPIN,  //Means front-most rollers are not spinning
  }

  enum e_IntakePositionState {
    EXTENDED,
    RETRACTED
  }


  enum e_IntakeDetectsImHolding {
    CONE,
    CUBE,
    NOTHING,
    ERROR
  }

  enum e_IntakeMotorState {
    PosSPIN,  //Means front-most side of rollers spinning INTO the Intake, toward it's center 
    NegSPIN,  //Means front-most side of rollers spinning OUT-OF the Intake, toward it's exterior
    NoSPIN,  //Means front-most rollers are not spinning
  }

  enum e_ArmShoulderPositionState {
    IsUP,  //Means Elevator is in up postion
    IsDOWN  // Means Elevator is in down position
  }

  enum e_ArmElbowPositionState {
    IsPICKUP,   //Means Arms is in position to receive game element from Intake
    IsHIGHCONE,  // Means Arm is in High Position to deposit cone
    IsHIGHCUBE,  // Means Arm is in High Position to deposit cube
    IsMIDCONE,  // Means Arm is in Mid Position to deposit cone
    IsMIDCUBE,  // Means Arm is in Mid Position to deposit cube
    IsFLOORCONE,  // Means Arm is in Floor High Position to deposit cone 
    IsFLOORCUBE,  // Means Arm is in Floor Position to deposit cube
  }

  public enum e_CurrentGameElementImWanting{
    CONE,
    CUBE,
    NONE
  }

  public enum e_CurrentGameElementImHolding{
    CONE,
    CUBE,
    NONE
  }


  private RobotState(RobotStateIO io) {
    this.io = io;
    m_Claw = Claw.getInstance();
    wantedLEDState = 0;
    isSolid = false;
    isFlashing = false;

    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(SignalingConstants.LEDLENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  // public void setSegment(int start, int end, int red, int green, int blue) {
  //     for (var i=start; i < end; i++)
  //     {
  //       m_ledBuffer.setRGB(i, red , green, blue);
  //     }
  // }

  public void setSegment(int start, int end, Color color) {
      for (var i=start; i < end; i++)
      {
        m_ledBuffer.setLED(i, color);
      }
      // m_led.setData(m_ledBuffer);
  }

  // public void setleftfrontlow(Status s) {
  //     leftfrontlowcolor = LookUpColor(s);
  //     leftfrontlowflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.LEFTFRONTLOWSTART, SignalingConstants.LEFTFRONTLOWSTOP, leftfrontlowcolor);
  //     m_led.setData(m_ledBuffer);
  // }
  
  // public void setleftfronthigh(Status s) {
  //     leftfronthighcolor = LookUpColor(s);
  //     leftfronthighflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.LEFTFRONTHIGHSTART, SignalingConstants.LEFTFRONTHIGHSTOP, leftfronthighcolor);
  //     m_led.setData(m_ledBuffer);
  // }
  
  // public void setcrossbarhigh(Status s) {
  //     crosshighcolor = LookUpColor(s);
  //     crosshighflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP, crosshighcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  // public void setrightfronthigh(Status s) {
  //     rightfronthighcolor = LookUpColor(s);
  //     rightfronthighflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.RIGHTFRONTHIGHSTART, SignalingConstants.RIGHTFRONTHIGHSTOP, rightfronthighcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  // public void setrightfrontlow(Status s) {
  //     rightfrontlowcolor = LookUpColor(s);
  //     rightfrontlowflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.RIGHTFRONTLOWSTART, SignalingConstants.RIGHTFRONTLOWSTOP, rightfrontlowcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  // public void setleftback(Status s) {
  //     leftbackcolor = LookUpColor(s);
  //     leftbackflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTOP, leftbackcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  // public void setcrossbarlow(Status s) {
  //     crosslowcolor = LookUpColor(s);
  //     crosslowflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.CROSSLOWSTART, SignalingConstants.CROSSLOWSTOP, crosslowcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  // public void setrightback(Status s) {
  //     rightbackcolor = LookUpColor(s);
  //     rightbackflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTOP, rightbackcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  public void setallyellow()
  {
    if (isFlashing)
    {
      allflash = true;
    }
    setSegment(0, 73, Color.kOrange);
    m_led.setData(m_ledBuffer);
  }

  public void setallpurple()
  {
    if (isFlashing)
    {
      allflash = true;
    }
    setSegment(0, 73, Color.kPurple);
    m_led.setData(m_ledBuffer);
  }

  public void setallred()
  {
    if (isFlashing)
    {
      allflash = true;
    }
    setSegment(0, 73, Color.kRed);
    m_led.setData(m_ledBuffer);
  }

  public void setallblack()
  {
    setSegment(0, 73, Color.kBlack);
    m_led.setData(m_ledBuffer);
  }

  public void setall(Status s)
  {
    allcolor = LookUpColor(s);
    allflash = LookUpFlash(s);
    setSegment(0, 73, allcolor);
    m_led.setData(m_ledBuffer);
  }

  private Color LookUpColor(Status s){
    Color c = Color.kBlack;
    switch(s){
      case CONE: c = Color.kOrange;
      break;
      case CUBE: c = Color.kPurple;
      break;
      case CONEFLASH: c = Color.kOrange;
      break;
      case CUBEFLASH: c = Color.kPurple;
      break;
      case NONE: c = Color.kBlack;
      break;
    }
    return c;
  }
  
  private Boolean LookUpFlash(Status s){
      return ((s == Status.CUBEFLASH) || (s == Status.CONEFLASH));
  }

  // public void FlashColor() {
  //   m_flashcounter++;
  //   if (m_flashcounter == 25){
  //     if (leftfrontlowflash){
  //       setSegment(SignalingConstants.LEFTFRONTLOWSTART, SignalingConstants.LEFTFRONTLOWSTOP,leftfrontlowcolor);
  //     }
  //     if (leftfronthighflash){
  //       setSegment(SignalingConstants.LEFTFRONTHIGHSTART, SignalingConstants.LEFTFRONTHIGHSTOP,leftfrontlowcolor);
  //     }
  //     if (crosshighflash){
  //       setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP,crosshighcolor);
  //     }
  //     if (rightfronthighflash){
  //       setSegment(SignalingConstants.RIGHTFRONTHIGHSTART, SignalingConstants.RIGHTFRONTHIGHSTOP, rightfronthighcolor);
  //     }
  //     if (rightfrontlowflash){
  //       setSegment(SignalingConstants.RIGHTFRONTLOWSTART, SignalingConstants.RIGHTFRONTLOWSTOP,rightfrontlowcolor);
  //     }
  //     if (leftbackflash){
  //       setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTOP,leftbackcolor);
  //     }
  //     if (crosslowflash){
  //       setSegment(SignalingConstants.CROSSLOWSTART, SignalingConstants.CROSSLOWSTOP, crosslowcolor);
  //     }
  //     if (rightbackflash){
  //       setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTOP,rightbackcolor);
  //     }
  //    m_led.setData(m_ledBuffer);
  //    }
  //   if (m_flashcounter == 50){
  //     if (leftfrontlowflash){
  //       setSegment(SignalingConstants.LEFTFRONTLOWSTART, SignalingConstants.LEFTFRONTLOWSTOP,Color.kBlack);
  //     }
  //     if (leftfronthighflash){
  //       setSegment(SignalingConstants.LEFTFRONTHIGHSTART, SignalingConstants.LEFTFRONTHIGHSTOP,Color.kBlack);
  //     }
  //     if (crosshighflash){
  //       setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP,Color.kBlack);
  //     }
  //     if (rightfronthighflash){
  //       setSegment(SignalingConstants.RIGHTFRONTHIGHSTART, SignalingConstants.RIGHTFRONTHIGHSTOP, Color.kBlack);
  //     }
  //     if (rightfrontlowflash){
  //       setSegment(SignalingConstants.RIGHTFRONTLOWSTART, SignalingConstants.RIGHTFRONTLOWSTOP,Color.kBlack);
  //     }
  //     if (leftbackflash){
  //       setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTOP,Color.kBlack);
  //     }
  //     if (crosslowflash){
  //       setSegment(SignalingConstants.CROSSLOWSTART, SignalingConstants.CROSSLOWSTOP, Color.kBlack);
  //     }
  //     if (rightbackflash){
  //       setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTOP,Color.kBlack);
  //     }
  //     m_led.setData(m_ledBuffer);
  //     m_flashcounter = 0;
  //   }
  // }

  public void flashAll()
  {
    if (m_flashcounter == 10)
    {
      if (allflash = true)
      {
        if (wantedLEDState == 1)
        {
          setallyellow();
        }
        else if (wantedLEDState == 2)
        {
          setallpurple();
        }
        else if (wantedLEDState == 3)
        {
          setallred();
        }
      }
      m_led.setData(m_ledBuffer);
    }
    if (m_flashcounter == 20)
    {
      if (allflash = true)
      {
        setallblack();
      }
      m_led.setData(m_ledBuffer);
      m_flashcounter = 0;
    }
    m_flashcounter++;
  }


  public void update() {
    if (DriverStation.isFMSAttached() && (alliance == null)) {
      alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }
  }

  private void setColorWantState(int LEDState) {
    //System.out.println("WAS CALLED");
    wantedLEDState = LEDState;
    if (wantedLEDState == 0) {
      isFlashing = false;
      setallblack();
    }
    else if (wantedLEDState == 1) {
      isFlashing = true;
      setallyellow();
    }
    else if (wantedLEDState == 2) {
      isFlashing = true;
      setallpurple();
    }
    else if (wantedLEDState == 3) {
      isFlashing = true;
      setallred();
    }
  }

  // public Command setColorWantStateCommand(int LEDState)
  // {
  //   System.out.println("setColorWantStateCommand()");
  //   return this.runOnce(() -> setColorWantState(LEDState));
  // }

  public static RobotState getInstance() {
    if(instance == null) {instance = new RobotState(new RobotStateIO() {});}
    return instance;
  }

  public Command setColorWantedState(int state) {
    return this.runOnce(() -> this.setColorWantState(state)).withName("setColorWantedState");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);

    //(m_Claw.getLinebreakOne() == false || m_Claw.getLinebreakTwo() == false)
    if (!m_Claw.getIsLinebreakOne() || !m_Claw.getIsLinebreakTwo()) {
      isSolid = true;
      isFlashing = false;
      if (wantedLEDState == 1) {
        setallyellow();
      }
      else if (wantedLEDState == 2 || wantedLEDState == 3) {
        setallpurple();
      }
    }
    else if ((m_Claw.getIsLinebreakOne() && m_Claw.getIsLinebreakTwo()) && isSolid)
    {
      setallblack();
      wantedLEDState = 0;
      isSolid = false;
    }
    else if (!isSolid && isFlashing){
      // System.out.println("isSolid == false and isFlashing = true");
      // if (wantedLEDState == 1)
      // {
      //   setallyellow();
      // }
      // else if (wantedLEDState == 2)
      // {
      //   setallpurple();
      // }
      if (wantedLEDState == 0)
      {
        allflash = false;
      }
      flashAll();
    }
  }

}