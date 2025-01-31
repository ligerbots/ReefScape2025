// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
// import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    private final int LED_PWM_PORT = 0;
    
    // Our LED strip has a density of 60 LEDs per meter
    private static final Distance LED_SPACING = Meters.of(1 / 60.0);
    
    private static final int NUM_LEDS = 60;

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    
    private LEDPattern pattern = LEDPattern.solid(Color.kBlack);

    // Creates a new Leds.
    public Leds() {
        // PWM port 0
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(LED_PWM_PORT);
        
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
        m_led.setLength(m_ledBuffer.getLength());
        
        m_led.start();
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Update the buffer with the pattern
        pattern.applyTo(m_ledBuffer);

        // Set the LEDs
        m_led.setData(m_ledBuffer);
    }

    // Colors include Color.kRed, Color.kOrange, Color.kYellow, Color.kGreen
    public void setSolidPattern(Color c) {
        pattern = LEDPattern.solid(c);
    }

    public void setBlinkPattern(Color c) {
        pattern = LEDPattern.solid(c).blink(Seconds.of(1.5));
    }

    public void setRainbowPattern() {
        pattern = LEDPattern.rainbow(255, 128);
    }

    public void setRainbowScrollingPattern() {
        pattern = LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);
    }

    // Percentage is 0 to 1, utilizes last used color
    public void setBarPattern(double percentage) {
        pattern = LEDPattern.progressMaskLayer(() -> percentage).mask(pattern);
    }
}
