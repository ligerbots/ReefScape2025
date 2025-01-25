// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  /** Creates a new Leds. */

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private final int LED_PWM_PORT = 0;

 // Our LED strip has a density of 60 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 60.0);

  LEDPattern m_scrollingPattern;

  public Leds() {

    // PWM port 0
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(LED_PWM_PORT);

      // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);

   m_led.setLength(m_ledBuffer.getLength());

    // Create an LED pattern that sets the entire strip to solid red
    LEDPattern pattern = LEDPattern.solid(Color.kBlue);

    // Apply the LED pattern to the data buffer
    pattern.applyTo(m_ledBuffer);

    // all hues at maximum saturation and half brightness
    pattern = LEDPattern.rainbow(255, 128);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // of 1 meter per second.
    m_scrollingPattern = pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    // Apply the LED pattern to the data buffer
    m_scrollingPattern.applyTo(m_ledBuffer);

     // Write the data to the LED strip
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        // Update the buffer with the rainbow animation
        m_scrollingPattern.applyTo(m_ledBuffer);
        // Set the LEDs
        m_led.setData(m_ledBuffer);
    
  }
}
