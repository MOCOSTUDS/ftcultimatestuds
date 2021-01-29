package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StudArm {

    public DcMotor arm = null;
    public DigitalChannel digIn1;
    public CRServo servoLoop = null;
    private float armAngle = 0;

    public float arm_zero_position=0;

    public void init(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        digIn1 = hardwareMap.get(DigitalChannel.class, "switch1"); //false is not pressed
        digIn1.setMode(DigitalChannel.Mode.INPUT);
        arm_zero_position = arm.getCurrentPosition();
        servoLoop  = hardwareMap.crservo.get("servo_loop");


        findZeroPoint();
    }

    public void findZeroPoint() {
        /*
       arm.setPower(0.2);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //going up
        arm.setPower(-0.3);
        while(digIn1.getState()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        arm.setPower(0);

         */
        arm_zero_position=arm.getCurrentPosition();
    }






    public int getarmPosition() {
        return ( (int)(arm.getCurrentPosition() - arm_zero_position));
    }

    public boolean readyToDrop() {
        if (arm.getCurrentPosition() - arm_zero_position > 500)
            return true;
        return false;
    }

    public boolean inStartingPostion() {
        if (arm.getCurrentPosition() - arm_zero_position < 100)
            return true;
        return false;
    }

    public void movearm( double powerVal) {
        if (digIn1.getState()== false && powerVal < 0) {
            arm.setPower(0);
        }
        else if(Math.abs(powerVal) < 0.1) {
            arm.setPower(0);
        }else{
            arm.setPower(powerVal);
        }
    }

    public void moveToDrop(){
        arm.setPower(0.2);
        while (!readyToDrop()){

            //telemetry.addData("armPostion", getarmPosition());
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        arm.setPower(0);

    }
    public void openLoop(){
        servoLoop.setPower(1);
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();

        }
        servoLoop.setPower(0);

    }
    public void closeLoop() {
        servoLoop.setPower(-1);
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();

        }
        servoLoop.setPower(0);
    }
    public void moveToSwitch(){
        arm.setPower(-0.2);
        while (!inStartingPostion()){
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        arm.setPower(0);

    }
    public void dropBobber(){
        moveToDrop();
        openLoop();
        moveToSwitch();
    }
}


