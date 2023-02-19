package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

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
    private boolean flashFirstTime = true;
    private long    flashEnd = 0;

    private double  previousCode = 0;
    private double  ledCode      = 0.41;

    // Objects
    private Spark ledController;

    private LED() {
        ledController = new Spark(LED_PWM);
    }

    public void updateLED() {
        if (previousCode != ledCode) {
            ledController.set(ledCode);
        }
        previousCode = ledCode;
    }

    public void teamColors() {
        ledCode = 0.41;
    }

    public void flashCubeOn() {
        ledCode = 0.91;
    }

    public void flashCubeOff() {
        ledCode = 0.87;
    }

    public void flashConeOn() {
        ledCode = 0.69;
    }

    public void flashConeOff() {
        ledCode = 0.65;
    }
}
