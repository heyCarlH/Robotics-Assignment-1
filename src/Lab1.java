import java.util.LinkedList;
import java.util.Queue;

import lejos.hardware.*;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

public class Lab1 {
public static void main (String args[]) {
System.out.println("Press any key to start");
        Button.waitForAnyPress();

        // create two motor objects to control the motors.
        EV3MediumRegulatedMotor motorA = new EV3MediumRegulatedMotor(MotorPort.A);
        EV3MediumRegulatedMotor motorB = new EV3MediumRegulatedMotor(MotorPort.B);
        
        motorA.setSpeed(400);
        motorB.setSpeed(400);
        
        // move forward 1.2m
        motorA.synchronizeWith(new EV3MediumRegulatedMotor[] {motorB});
        motorA.startSynchronization();
        
        motorA.forward();
        motorB.forward();
        
        motorA.endSynchronization();

        Delay.msDelay(6366); //+250 +12

        // stop motors with brakes on
        motorA.stop(true);
        motorB.stop(true);
 
        Sound.beepSequence(); // Objective 1 done!
        
        System.out.println("Press any key to start");
        Button.waitForAnyPress();
        
        // set up ultrasensor
        EV3UltrasonicSensor ultrasensor = new EV3UltrasonicSensor(SensorPort.S4);
        SensorMode sonic = (SensorMode) ultrasensor.getDistanceMode();
        float[] sample_sonic = new float[sonic.sampleSize()];
        
        // keep moving the robot       
        motorA.synchronizeWith(new EV3MediumRegulatedMotor[] {motorB});
        motorA.startSynchronization();
        
        motorA.forward();
        motorB.forward();
        
        motorA.endSynchronization();
        
        // set up the moving average of the ultrasonic sensor
        Queue<Float> lastThreeSamples = new LinkedList<>();
        float lastThreeSum = 0;
        for (int i = 0; i < 3; i++) {
        sonic.fetchSample(sample_sonic, 0);
            lastThreeSamples.add(sample_sonic[0]);
            lastThreeSum += sample_sonic[0];
        }
        float movingAverage = (float)lastThreeSum/3;
        
        // keep fetching the distance to the obstacle if it's more than 0.5m away
        while(movingAverage > 0.55f) {
          lastThreeSum -= lastThreeSamples.poll();
    sonic.fetchSample(sample_sonic, 0);
    lastThreeSamples.add(sample_sonic[0]);
    lastThreeSum += sample_sonic[0];
    movingAverage = lastThreeSum/3;
      }
        
        // stop motors with brakes on 0.5m before the obstacle 
        motorA.stop(true);
        motorB.stop(true);
        
        Sound.beepSequence(); // Objective 2 done!
        
        System.out.println("Press any key to start");
        Button.waitForAnyPress();
        
        // set up bump sensor
        EV3TouchSensor touchsensor = new EV3TouchSensor(SensorPort.S1);
        SensorMode touch = touchsensor.getTouchMode();
        float[] sample_touch = new float[touch.sampleSize()];
        EV3TouchSensor touchsensor2 = new EV3TouchSensor(SensorPort.S3);
        SensorMode touch2 = touchsensor2.getTouchMode();
        float[] sample_touch2 = new float[touch2.sampleSize()];
        
        // keep moving the robot       
        motorA.startSynchronization();
        
        motorA.forward();
        motorB.forward();
        
        motorA.endSynchronization();
        
        touch.fetchSample(sample_touch, 0);
        touch2.fetchSample(sample_touch2, 0);
        
        // keep fetching data from the bump sensor
        while(sample_touch[0] == 0.0 && sample_touch2[0] == 0.0) { 
    touch.fetchSample(sample_touch, 0);
    touch2.fetchSample(sample_touch2, 0);
      }
        
        // stop motors with brakes on if bumped into something 
        motorA.stop(true);
        motorB.stop(true);
        
        // move the robot back 0.5m        
        motorA.startSynchronization();
        
        motorA.backward();
        motorB.backward();
        
        motorA.endSynchronization();
        
        Delay.msDelay(2653);

        // stop motors with brakes on. 
        motorA.stop(true);
        motorB.stop(true);

        // free up motor resources - all objectives done!
        motorA.close(); 
        motorB.close();        
}
}