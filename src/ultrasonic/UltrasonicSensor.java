package ultrasonic;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class UltrasonicSensor {

    public static void main(String[] args) {

        EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);
        SampleProvider distance = usSensor.getDistanceMode();
        float[] sample = new float[distance.sampleSize()];

        // PID constants
        float Kp = 800; // Proportional gain
        float Ki = 0;   // (Optional) Integral gain
        float Kd = 300; // Derivative gain

        float targetDistance = 0.25f; // desired distance from the object
        float error, previousError = 0, integral = 0, derivative, correction;

        LCD.drawString("Press any button", 0, 0);
        Button.waitForAnyPress();
        LCD.clear();

        Motor.B.setSpeed(300);
        Motor.C.setSpeed(300);
        Motor.B.forward();
        Motor.C.forward();

        while (!Button.ESCAPE.isDown()) {
            distance.fetchSample(sample, 0);
            float dist = sample[0]; // distance in meters

            LCD.clear();
            LCD.drawString("Distance: " + dist, 0, 0);

            // PID calculations
            error = targetDistance - dist;
            integral += error;
            derivative = error - previousError;

            correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

            previousError = error;

            // Adjust motor speeds based on correction
            int baseSpeed = 300;
            int leftSpeed = (int)(baseSpeed + correction);
            int rightSpeed = (int)(baseSpeed - correction);

            // Clamp speeds to avoid errors
            leftSpeed = Math.max(100, Math.min(600, leftSpeed));
            rightSpeed = Math.max(100, Math.min(600, rightSpeed));

            Motor.B.setSpeed(leftSpeed);
            Motor.C.setSpeed(rightSpeed);

            Motor.B.forward();
            Motor.C.forward();

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        Motor.B.stop(true);
        Motor.C.stop();
        usSensor.close();
        LCD.clear();
        LCD.drawString("Task Finished", 0, 0);
        Button.waitForAnyPress();
    }
}
