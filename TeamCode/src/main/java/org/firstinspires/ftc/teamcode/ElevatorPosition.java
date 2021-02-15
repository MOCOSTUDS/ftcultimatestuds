package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class ElevatorPosition implements Runnable{

    private StudElevator elevator;


    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double shooterEncoderValue;
    double lastShooterEncoderValue = 0.0;
    long lastMeasuredTime = 0;

    private double targetLocation;
    private double currentLocation;

    private String mode = "IDLE";

    public double pValue = 0.1;
    public double iValue = 0.02;
    public double dValue = 0.00;

    public double movingPower = 0.9;
    public double reversePower = -0.3;

    public miniPID pid;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;


    public ElevatorPosition(StudElevator ele, int threadSleepDelay){
        this.elevator = ele;
        sleepTime = threadSleepDelay;
        pid = new miniPID(pValue, iValue, dValue);
        pid.setOutputLimits(1);
        pid.setSetpointRange(0.8);
    }






    public void setMovingMode() {
        isRunning = true;
        mode = "MOVING";
    }

    public void stop(){
        isRunning = false;
        lastShooterEncoderValue = 0.0;
        lastMeasuredTime = 0;
        mode = "IDLE";
    }

    public void setIdle() {
        isRunning = true;
        mode = "IDLE";
    }
    @Override
    public void run() {
        while(isRunning) {
            if (mode.equals("MOVING")) {
                if (elevator.readyToShoot()) {
                    elevator.servoBlock.setPosition(0.3);
                } else {
                    //shooterUpdate.setBackMode();
                    elevator.servoBlock.setPosition(-0.5);
                }
                elevator.movePID();
            } else if (mode.equals("IDLE")) {
                elevator.stop();
            }
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }


}

