// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * Start of the LED class
 */
public class LED {
    // Singleton for LedLights because it is used in many places
	public static LED instance = null;
	public static synchronized LED getInstance() {
		if (instance == null) {
			instance = new LED();
		}

		return instance;
	}

    // Constants
    private final int LED_PWM = 0;

    // Variables
    private double  previousCode = 0;
    private double  ledCode      = 0.41;

    // Objects
    private Spark ledController;

    /**
     * The constructor for the LED class
     */
    private LED() {
        ledController = new Spark(LED_PWM);
        ledController.set(0.41);
    }

    /**
     * A method to constantly update the LED's
     */
    public void updateLED() {
        if (previousCode != ledCode) {
            ledController.set(ledCode);
        }
        previousCode = ledCode;
    }

    /**
     * A function to change the LED's to our team colors (blue and gold)
     */
    public void teamColors() {
        ledCode = 0.41;
    }

    /**
     * A function to change the LED's to purple
     */
    public void flashCubeOn() {
        ledCode = 0.91;
    }

    /**
     * A function to change the LED's to blue
     */
    public void flashCubeOff() {
        ledCode = 0.87;
    }

    /**
     * A function to change the LED's to yellow
     */
    public void flashConeOn() {
        ledCode = 0.69;
    }

    /**
     * A function to change the LED's to orange
     */
    public void flashConeOff() {
        ledCode = 0.65;
    }
}

// End of the LED class