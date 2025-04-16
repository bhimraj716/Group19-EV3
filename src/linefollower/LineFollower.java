package linefollower;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {

    public static void main(String[] args) {
        // Initialize color sensor on port S4 in red mode
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
        SampleProvider light = colorSensor.getRedMode();
        float[] sample = new float[light.sampleSize()];

        // Motor setup
        int baseSpeed = 300;
        int turnSpeed = 150;
        Motor.A.setSpeed(baseSpeed);
        Motor.B.setSpeed(baseSpeed);
        Motor.A.forward();
        Motor.B.forward();

        // Threshold for black line detection (tune this value based on testing)
        float threshold = 0.2f;

        // Main loop
        while (!Button.ESCAPE.isDown()) {
            // Read sensor value
            light.fetchSample(sample, 0);
            float lightValue = sample[0];

            // Display light intensity for debugging
            LCD.clear();
            LCD.drawString("Light: " + (int)(lightValue * 100) + "%", 0, 0);

            if (lightValue < threshold) {
                // ON the black line -> go forward
                Motor.A.setSpeed(baseSpeed);
                Motor.B.setSpeed(baseSpeed);
            } else {
                // OFF the line -> adjust
                // Simple logic: turn slightly right to search for the line
                Motor.A.setSpeed(turnSpeed);  // Slow left motor
                Motor.B.setSpeed(baseSpeed);  // Fast right motor
            }

            // Apply forward motion with updated speeds
            Motor.A.forward();
            Motor.B.forward();

            // Small delay to stabilize updates
            Delay.msDelay(50);
        }

        // Stop motors and close sensor on exit
        Motor.A.stop(true);
        Motor.B.stop();
        colorSensor.close();
    }
}
