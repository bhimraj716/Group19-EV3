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

        // Color sensor setup
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
        SampleProvider light = colorSensor.getRGBMode();
        float[] sample = new float[light.sampleSize()];

        // Basic setup
        int baseSpeed = 250; // Base speed for both motors
        int maxSpeed = 600;
        int minSpeed = 100;
        // PID control constants
        float Kp = 600; // Proportional gain
        float Ki = 0; // Integral gain
        float Kd = 150; // Derivative gain

        float target = 0.2f; // Target reflectance (tune based on your line color)
        float integral = 0;
        float lastError = 0;

        LCD.drawString("Line Follower PID", 0, 0);
        Delay.msDelay(1000);

        // Main loop
        while (!Button.ESCAPE.isDown()) {

            // Read sensor value
            light.fetchSample(sample, 0);
            float lightValue = sample[0];

            // Calculate PID components
            float error = lightValue - target;
            integral += error;
            float derivative = error - lastError;
            float correction = Kp * error + Ki * integral + Kd * derivative;

            lastError = error;

            // Calculate motor speeds using differential drive
            int leftSpeed = (int) (baseSpeed + correction);
            int rightSpeed = (int) (baseSpeed - correction);

            // Clamp speeds to motor limits
            leftSpeed = Math.max(minSpeed, Math.min(maxSpeed, leftSpeed));
            rightSpeed = Math.max(minSpeed, Math.min(maxSpeed, rightSpeed));

            // Set speeds and direction
            Motor.A.setSpeed(leftSpeed);
            Motor.B.setSpeed(rightSpeed);
            Motor.A.forward();
            Motor.B.forward();

            // Display debug info
            LCD.clear();
            LCD.drawString("Light: " + (int) (lightValue * 100) + "%", 0, 1);
            LCD.drawString("L: " + leftSpeed + " R: " + rightSpeed, 0, 2);

            Delay.msDelay(50);
        }

        // Stop the robot
        Motor.A.stop();
        Motor.B.stop();
        colorSensor.close();
    }
}
