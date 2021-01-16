package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class ShooterBeltSpeedVelocity implements Runnable{

    private DcMotor shooterMotor;


    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double shooterEncoderValue;
    double lastShooterEncoderValue = 0.0;
    long lastMeasuredTime = 0;

    private double targetSpeed;
    private double currentSpeed;
    private double lastPower;
    private long lastDelta;

    private String mode = "IDLE";

    public double pValue = 0.1;
    public double iValue = 0.02;
    public double dValue = 0.00;

    public double cruisingPower = 0.9;
    public double reversePower = -0.3;

    public miniPID pid;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public long getLastTime() {
        return lastDelta;
    }

    public ShooterBeltSpeedVelocity(StudShooter shooter, int threadSleepDelay){
        this.shooterMotor = shooter.getShooterMotor();
        sleepTime = threadSleepDelay;
        pid = new miniPID(pValue, iValue, dValue);
        pid.setOutputLimits(1);
        pid.setSetpointRange(0.8);
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void velocityUpdate(){
        long currentTime = System.currentTimeMillis();
        //Get Current Positions
        shooterEncoderValue = shooterMotor.getCurrentPosition();
        if (lastShooterEncoderValue > 0) {
            double change = shooterEncoderValue - lastShooterEncoderValue;
            long deltaT = (currentTime - lastMeasuredTime) ;
            lastDelta = deltaT;
            currentSpeed = change/deltaT;
        }
        lastShooterEncoderValue = shooterEncoderValue;
        lastMeasuredTime = currentTime;
    }

    public void  maintainSpeedModeStart(double targetSpeed) {
        this.targetSpeed = targetSpeed;
        // reset PI Params
        this.mode = "SHOOTING";
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnVelocity(){ return currentSpeed; }

    public double returnPower(){ return lastPower; }

    /**
     * Stops the position update thread
     */
    public void stop(){
        isRunning = false;
        lastShooterEncoderValue = 0.0;
        lastMeasuredTime = 0;
        mode = "IDLE";
    }

    public void setBackMode() {
        mode = "BACKWARDS";
    }


    @Override
    public void run() {
        while(isRunning) {
            velocityUpdate();
            if (mode.equals("SHOOTING")) {
                //shooterMotor.setPower(cruisingPower);
                // DO PID HERE
                double newPower=cruisingPower+pid.getOutput(currentSpeed, targetSpeed);
                //double newPower=cruisingPower;
                //double newPower = targetSpeed;
                shooterMotor.setPower(newPower);
                lastPower = newPower;
            } else if (mode.equals("REVERSE")) {
                if (currentSpeed < .1) {
                    shooterMotor.setPower(reversePower);
                    lastPower = reversePower;
                } else {
                    shooterMotor.setPower(0.0);
                    lastPower = reversePower;
                }
            } else if (mode.equals("IDLE")) {
                shooterMotor.setPower(0.0);
                lastPower = 0.0;
            }
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }


}

