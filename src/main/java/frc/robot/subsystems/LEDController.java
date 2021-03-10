package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * @author Daniel Pearson
 * @version 2/20/2020
 */
public class LEDController extends SubsystemBase {
  
  public enum LEDColors {
    RED, GREEN, BLUE, ORANGE
  }
  private final AddressableLED ledStrip = new AddressableLED(Constants.LEDSTRIP);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
  
  public LEDController() { 
  ledStrip.setLength(ledBuffer.getLength());
  
  ledStrip.setData(ledBuffer);
  ledStrip.start();
  }

  @Override
  public void periodic() {
  }  

  public synchronized void setColor(int r, int g, int b){
    for(int i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, r, g, b); 
    }

    ledStrip.setData(ledBuffer);
  }

  public synchronized void setColorEnum(LEDColors color){
    switch(color) {
      case RED: 
        this.setColor(245, 22, 52);
        break;
      case GREEN:
        this.setColor(0, 209, 31);
        break;
      case BLUE:
        this.setColor(0, 188, 209);
        break;
      case ORANGE:
        this.setColor(252, 186, 3);
        break;
    }
  }
}
