package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class LightSubsystem extends SubsystemBase {

  
    private static final int kPort = 9;
    private static final int kLength = 120;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_buffer;
   

  // Below is the Rainbow Pattern 
      private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
      private static final Distance kLedSpacing = Meters.of(1 / 120.0);
      private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
  // End Rainbow Pattern


  // Start Red Pattern 
     private LEDPattern red = LEDPattern.solid(Color.kRed);
  // End Red Pattern 


  // Start Blue Pattern
      private LEDPattern blue = LEDPattern.solid(Color.kBlue);
  // End Blue Pattern


    public LightSubsystem() {
      m_led = new AddressableLED(kPort);
      m_buffer = new AddressableLEDBuffer(kLength);
      m_led.setLength(kLength);
      m_led.start();
  

      // Set the default command to turn the strip off, otherwise the last colors written by
      // the last command to run will continue to be displayed.
      // Note: Other default patterns could be used instead!
      setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
    }
  
    @Override
    public void periodic() {
      // Periodically send the latest LED color data to the LED strip for it to display
      m_led.setData(m_buffer);
    }
    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
      return run(() -> pattern.applyTo(m_buffer));
    }

    public void setRed()
    {
      red.applyTo(m_buffer);
      m_led.setData(m_buffer);
    }

    public void setBlue()
    {
      blue.applyTo(m_buffer);
      m_led.setData(m_buffer);
    }
  }
