package ultrasonic;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class UltrasonicSensor {

    public static void main(String[] args) {

        // Initialize the ultrasonic sensor on port 2
        EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);
        SampleProvider distance = usSensor.getDistanceMode();
        float[] sample = new float[distance.sampleSize()];

        // Wait for a button press to start
        LCD.drawString("Press any button", 0, 0);
        Button.waitForAnyPress();
        LCD.clear();

        // Move forward at full speed
        Motor.B.forward();
        Motor.C.forward();

        while (true) {
            distance.fetchSample(sample, 0);
            float dist = sample[0]; // distance in meters

            // Display distance
            LCD.clear();
            LCD.drawString("Distance: " + dist + " m", 0, 0);

            // If an obstacle is detected within 0.3 meters
            if (dist < 0.3) {
                // Slow down
                Motor.B.setSpeed(100);  // Lower speed
                Motor.C.setSpeed(100);
                Motor.B.forward();
                Motor.C.forward();

                // If the object is very close (e.g., 0.15m), stop and return
                if (dist < 0.15) {
                    Motor.B.stop(true);
                    Motor.C.stop();

                    // Wait a moment
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    // Reverse back to starting point
                    Motor.B.setSpeed(300);
                    Motor.C.setSpeed(300);
                    Motor.B.backward();
                    Motor.C.backward();

                    // Move backward for 2 seconds (approx same distance)
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    Motor.B.stop(true);
                    Motor.C.stop();
                    break;
                }
            } else {
                // Move forward at normal speed
                Motor.B.setSpeed(400);
                Motor.C.setSpeed(400);
                Motor.B.forward();
                Motor.C.forward();
            }

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        usSensor.close();
        LCD.clear();
        LCD.drawString("Task Finished", 0, 0);
        Button.waitForAnyPress();
    }
}
