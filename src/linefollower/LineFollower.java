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

        // Initialize color sensor on port S4 using reflected red light
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
        SampleProvider light = colorSensor.getRedMode();  // or getRGBMode() if needed
        float[] sample = new float[light.sampleSize()];

        // Motor speed settings
        int baseSpeed = 250;
        int maxSpeed = 600;
        int minSpeed = 100;

        // PID control constants (tune for your robot)
        float Kp = 600;
        float Ki = 0;
        float Kd = 150;

        // Target reflectance value (black line should be near 0.2 or lower)
        float target = 0.2f;
        float integral = 0;
        float lastError = 0;

        // Display title
        LCD.drawString("PID Line Follower", 0, 0);
        Delay.msDelay(1000);

        // Main loop
        while (!Button.ESCAPE.isDown()) {
            // Read sensor value
            light.fetchSample(sample, 0);
            float lightValue = sample[0];

            // PID calculation
            float error = lightValue - target;
            integral += error;
            float derivative = error - lastError;
            float correction = Kp * error + Ki * integral + Kd * derivative;
            lastError = error;

            // Compute motor speeds
            int leftSpeed = (int) (baseSpeed + correction);
            int rightSpeed = (int) (baseSpeed - correction);

            // Clamp motor speeds
            leftSpeed = Math.max(minSpeed, Math.min(maxSpeed, leftSpeed));
            rightSpeed = Math.max(minSpeed, Math.min(maxSpeed, rightSpeed));

            // Set motor speeds and move forward
            Motor.A.setSpeed(leftSpeed);
            Motor.B.setSpeed(rightSpeed);
            Motor.A.forward();
            Motor.B.forward();

            // Debug display
            LCD.clear();
            LCD.drawString("Light: " + (int)(lightValue * 100) + "%", 0, 1);
            LCD.drawString("L:" + leftSpeed + " R:" + rightSpeed, 0, 2);

            Delay.msDelay(50);
        }

        // Stop and cleanup
        Motor.A.stop();
        Motor.B.stop();
        colorSensor.close();
    }
}
