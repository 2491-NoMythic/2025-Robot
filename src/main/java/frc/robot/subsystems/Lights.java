// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package
// frc.robot.subsystems;import edu.wpi.first.wpilibj2.command.SubsystemBase;
package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.BeanSerializer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.ElevatorEnums;
import frc.robot.settings.LightsEnums;
import frc.robot.settings.Constants.LightConstants;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

public class Lights extends SubsystemBase {
  /** Creates a new SubsystemLights. */
  private AddressableLED lights;
  CANdle candle;
  CANdleConfiguration CANdleConfig;

  private AddressableLEDBuffer LEDBuffer;
  private Timer timer;
  boolean blinkLights;
  LightsEnums lightsToBlink;
  int blinkedRed;
  int blinkedGreen;
  int blinkedBlue;

  public Lights() {
    lights = new AddressableLED(6);
    LEDBuffer = new AddressableLEDBuffer(60);
    lights.setLength(60);
    timer = new Timer();
    candle = new CANdle(LightConstants.CANDLE_ID);
    CANdleConfig = new CANdleConfiguration();
    CANdleConfig.v5Enabled = true;
    CANdleConfig.brightnessScalar = 1;
    candle.configAllSettings(CANdleConfig);
  }

  public void dataSetter() {
    lights.setData(LEDBuffer);
    lights.start();
  }

  /**
   * @param index the number (from 0 to max) of the LED to set
   * @param R
   * @param G
   * @param B
   */
  public void setOneLightRGB(int index, int R, int G, int B) {
    LEDBuffer.setRGB(index, R, G, B);
  }

  public void setCandleLights(int start, int end, int R, int G, int B) {
    candle.setLEDs(R, G, B, 0, start, end - start);
  }

  public void setLights(int start, int end, int R, int G, int B) {
    for (int i = start; i < end; i++) {
      setOneLightRGB(i, R, G, B);
    }
  }

  public void setAllLights(int R, int G, int B) {
    setLights(LightConstants.TOTAL_LIGHTS_RIO_STRIP_START, LightConstants.TOTAL_LIGHTS_RIO_STRIP_END, R, G, B);
    setCandleLights(LightConstants.TOTAL_LIGHTS_CANDLE_STRIP_START, LightConstants.TOTAL_LIGHTS_CANDLE_STRIP_END, R, G, B);
  }

  /** turns the lights off */
  public void lightsOut() {
    setLights(0, LEDBuffer.getLength(), 0, 0, 0);
    candle.setLEDs(0, 0, 0);
  }

  /**
   * sets a specific section of lights to a certain color. The sections to choose from are the ones in the LightsEnum. The sections lenghts are defined in Constnats
   * @param lightEnums the enum for the section of lights to set
   * @param R the Red value (0-255)
   * @param G the Green value (0-255)
   * @param B the Blue value (0-255)
   */
  public void setSystemLights(LightsEnums lightEnums, int R, int G, int B) {
    switch (lightEnums) {
      case ElevatorLeftAlgaeIndicator:
        setCandleLights(LightConstants.LEFT_ELEVATOR_LIGHTS_ALGAE_START, LightConstants.LEFT_ELEVATOR_LIGHTS_ALGAE_END,
            R, G, B);
        break;
      case ElevatorLeft1:
        setCandleLights(LightConstants.LEFT_ELEVATOR_LIGHTS_1_START, LightConstants.LEFT_ELEVATOR_LIGHTS_1_END, R, G, B);
        break;
      case ElevatorLeft2:
        setCandleLights(LightConstants.LEFT_ELEVATOR_LIGHTS_1_END + 1, LightConstants.LEFT_ELEVATOR_LIGHTS_2_END, R, G, B);
        break;
      case ElevatorLeft3:
        setCandleLights(LightConstants.LEFT_ELEVATOR_LIGHTS_2_END + 1, LightConstants.LEFT_ELEVATOR_LIGHTS_3_END, R, G, B);
        break;
      case ElevatorLeft4:
        setCandleLights(LightConstants.LEFT_ELEVATOR_LIGHTS_3_END + 1, LightConstants.LEFT_ELEVATOR_LIGHTS_4_END, R, G, B);
        break;
      case ElevatorLeft5:
        setCandleLights(LightConstants.LEFT_ELEVATOR_LIGHTS_4_END + 1, LightConstants.LEFT_ELEVATOR_LIGHTS_5_END, R, G, B);
        break;
      case ElevatorRightAlgaeIndicator:
        setCandleLights(LightConstants.RIGHT_ELEVATOR_LIGHTS_ALGAE_START,
            LightConstants.RIGHT_ELEVATOR_LIGHTS_ALGAE_END, R, G, B);
        break;
      case ElevatorRight1:
        setCandleLights(LightConstants.RIGHT_ELEVATOR_LIGHTS_2_END + 1, LightConstants.RIGHT_ELEVATOR_LIGHTS_1_END, R, G, B);
        break;
      case ElevatorRight2:
        setCandleLights(LightConstants.RIGHT_ELEVATOR_LIGHTS_3_END + 1, LightConstants.RIGHT_ELEVATOR_LIGHTS_2_END, R, G, B);
        break;
      case ElevatorRight3:
        setCandleLights(LightConstants.RIGHT_ELEVATOR_LIGHTS_4_END + 1, LightConstants.RIGHT_ELEVATOR_LIGHTS_3_END, R, G, B);
        break;
      case ElevatorRight4:
        setCandleLights(LightConstants.RIGHT_ELEVATOR_LIGHTS_5_END + 1, LightConstants.RIGHT_ELEVATOR_LIGHTS_4_END, R, G, B);
        break;
      case ElevatorRight5:
        setCandleLights(LightConstants.RIGHT_ELEVATOR_LIGHTS_5_START, LightConstants.RIGHT_ELEVATOR_LIGHTS_5_END, R, G, B);
        break;
      case Funnel:
        setLights(LightConstants.FUNNEL_LIGHTS_START, LightConstants.FUNNEL_LIGHTS_END, R, G, B);
        break;
      case Drivetrain:
        setCandleLights(LightConstants.DRIVETRAIN_LIGHTS_START, LightConstants.DRIVETRAIN_LIGHTS_END, R, G, B);
        break;
    }
  }

  public void setElevatorLevel(ElevatorEnums ElevatorHeight) {
    setSystemLights(LightsEnums.ElevatorLeft1, 100, 0, 0);
    setSystemLights(LightsEnums.ElevatorLeft2, 100, 0, 0);
    setSystemLights(LightsEnums.ElevatorLeft3, 100, 0, 0);
    setSystemLights(LightsEnums.ElevatorLeft4, 100, 0, 0);
    setSystemLights(LightsEnums.ElevatorRight1, 100, 0, 0);
    setSystemLights(LightsEnums.ElevatorRight2, 100, 0, 0);
    setSystemLights(LightsEnums.ElevatorRight3, 100, 0, 0);
    setSystemLights(LightsEnums.ElevatorRight4, 100, 0, 0);

    if (ElevatorHeight == ElevatorEnums.Reef1) {
      setSystemLights(LightsEnums.ElevatorLeft1, 0, 100, 0);
      setSystemLights(LightsEnums.ElevatorRight1, 0, 100, 0);
    }
    if (ElevatorHeight == ElevatorEnums.Reef2) {
      setSystemLights(LightsEnums.ElevatorLeft2, 0, 100, 0);
      setSystemLights(LightsEnums.ElevatorRight2, 0, 100, 0);
    }
    if (ElevatorHeight == ElevatorEnums.Reef3) {
      setSystemLights(LightsEnums.ElevatorLeft3, 0, 100, 0);
      setSystemLights(LightsEnums.ElevatorRight3, 0, 100, 0);
    }
    if (ElevatorHeight == ElevatorEnums.Reef4) {
      setSystemLights(LightsEnums.ElevatorLeft4, 0, 100, 0);
      setSystemLights(LightsEnums.ElevatorRight4, 0, 100, 0);
    }
  }

  /**
   * enter a section of lights that will be blinked on and off every half second
   * 
   * @param lightEnum
   * @param R
   * @param G
   * @param B
   */
  public void blinkLights(LightsEnums lightEnum, int R, int G, int B) {
    timer.start();
    lightsToBlink = lightEnum;
    blinkedBlue = B;
    blinkedGreen = G;
    blinkedRed = R;
    blinkLights = true;
  }

  /**
   * stops blinking whatever lights are blinking
   */
  public void stopBlinkingLights() {
    blinkLights = false;
    timer.stop();
    timer.reset();
  }

  private void updateBlinkedLights() {
    if (blinkLights) {
      if (timer.get() < 0.5) {
        setSystemLights(lightsToBlink, blinkedRed, blinkedGreen, blinkedBlue);
      } else if (timer.get() < 1) {
        setSystemLights(lightsToBlink, 0, 0, 0);
      } else {
        timer.reset();
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateBlinkedLights();
    dataSetter();
  }
}
