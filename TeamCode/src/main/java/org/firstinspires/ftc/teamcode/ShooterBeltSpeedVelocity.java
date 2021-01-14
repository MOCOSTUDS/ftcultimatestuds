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
    private double currentSpeed;
    private double lastSpeed;
    private double targetSpeed;

    private String mode = "IDLE";

    public double pValue = 0.4;
    public double iValue = 0.01;
    public double dValue = 0.02;

    public double sumErrror = 0.00;

    public double cruisingPower = 0.9;
    public double lastPower = cruisingPower;

    public miniPID pid;



    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;


    public ShooterBeltSpeedVelocity(StudShooter shooter, int threadSleepDelay){
        this.shooterMotor = shooter.getShooterMotor();

        sleepTime = threadSleepDelay;

        pid = new miniPID(pValue, iValue, dValue);
        pid.setOutputLimits(1);
        //pid.setMaxIOutput(1);
        //pid.setOutputRampRate(3);
        //pid.setOutputFilter(.3);
        pid.setSetpointRange(0.8);


    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void velocityUpdate(){
        long currentTime = System.currentTimeMillis();
        //Get Current Positions
        shooterEncoderValue = shooterMotor.getCurrentPosition();
        lastSpeed = currentSpeed;
        if (lastShooterEncoderValue > 0) {
            double change = shooterEncoderValue - lastShooterEncoderValue;
            double deltaT = (currentTime - lastMeasuredTime) ;
            currentSpeed = change/deltaT;
        }
        lastShooterEncoderValue = shooterEncoderValue;
        lastMeasuredTime = currentTime;
    }

    public void  maintainSpeedModeStart(double targetSpeed) {
        this.targetSpeed = targetSpeed;
        // reset PI Params
        sumErrror=0.0;
        this.mode = "SHOOTING";
    }




    public void  notShooting( double targetSpeed) {
        this.mode = "IDLE";
        this.targetSpeed = targetSpeed;
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

        shooterMotor.setPower(cruisingPower);
        lastPower = cruisingPower;
    }

    /**
     * Runs the thread

     public void run_old() {
     while(isRunning) {
     velocityUpdate();
     if (this.mode.equals("SHOOTING")) {
     // DO PID HERE
     double vError = currentSpeed - targetSpeed;
     sumErrror= sumErrror + vError;
     double newPower = cruisingPower - pValue*vError -iValue *sumErrror;
     if (newPower > 1.0)
     newPower = 1.0;
     shooterMotor.setPower(newPower);
     lastPower = newPower;

     }
     try {
     Thread.sleep(sleepTime);
     } catch (InterruptedException e) {
     e.printStackTrace();
     }
     }
     }
     */

    @Override
    public void run() {
        while(isRunning) {
            velocityUpdate();
            if (true/*this.mode.equals("SHOOTING")*/) {
                // DO PID HERE

                //double newPower=cruisingPower+pid.getOutput(currentSpeed, targetSpeed);
                //double newPower=cruisingPower;
                double newPower = targetSpeed;
                shooterMotor.setPower(newPower);
                lastPower = newPower;




            }
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }


}

