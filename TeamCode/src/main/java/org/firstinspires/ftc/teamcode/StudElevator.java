package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class StudElevator {

    public DcMotor elevator = null;
    public DigitalChannel digIn0;

    public Servo servoBlock = null;

    public float elevator_zero_position=0;
    public float targetElevator=0;


    public miniPID pid;



    public void init(HardwareMap hardwareMap) {
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator.setDirection(DcMotor.Direction.FORWARD);
        digIn0 = hardwareMap.get(DigitalChannel.class, "switch0"); //false is not pressed
        digIn0.setMode(DigitalChannel.Mode.INPUT);
        elevator_zero_position = elevator.getCurrentPosition();
        servoBlock  = hardwareMap.servo.get("shooter_block");
        servoBlock.setPosition(-0.5);
        findZeroPoint();

        pid = new miniPID(0.001, 0, 0);
        pid.setOutputLimits(1);
        pid.setSetpointRange(1000.0);

    }

    public void findZeroPoint() {
       elevator.setPower(0.4);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //going up
        elevator.setPower(-0.4);
        while(digIn0.getState()) {
            elevator.setPower(-0.4);
        }
        elevator.setPower(0);
        elevator_zero_position=elevator.getCurrentPosition();
    }

    public int getElevatorPosition() {

        return elevator.getCurrentPosition();
    }

    public boolean readyToShoot() {
        if (elevator.getCurrentPosition() - elevator_zero_position >2300) //previous number was 1800
            return true;
        return false;
    }

    public void moveElevator( double powerVal) {
        if (digIn0.getState() == false && powerVal < 0) {
            elevator.setPower(0);
        } else if (Math.abs(powerVal) < 0.1) {
            elevator.setPower(0);
        } else {
            elevator.setPower(powerVal);
        }
    }



    public void moveToShoot(double azum) {
        //going up

        if (elevator.getCurrentPosition() - elevator_zero_position < azum) {
            while (elevator.getCurrentPosition() - elevator_zero_position < azum) {

                elevator.setPower(0.8);

                if (readyToShoot()) {
                    servoBlock.setPosition(0.3);
                } else {
                    //shooterUpdate.setBackMode();
                    servoBlock.setPosition(-0.5);
                }

                if (elevator.getCurrentPosition() - elevator_zero_position > azum - 500) {
                    elevator.setPower(0.4);
                }

                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }else{
            while (elevator.getCurrentPosition() - elevator_zero_position > azum) {

                elevator.setPower(-0.8);

                if (readyToShoot()) {
                    servoBlock.setPosition(0.3);
                } else {
                    //shooterUpdate.setBackMode();
                    servoBlock.setPosition(-0.5);
                }

                if (elevator.getCurrentPosition() - elevator_zero_position < azum - 500) {
                    elevator.setPower(-0.4);
                }

                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }


        }
        elevator.setPower(0);

    }

    public void setTarget (float elevation) {

        targetElevator = elevation;
    }

    public double movePID (){

        pid.setPID(0.01,0.005,0.04);
        double vertical = pid.getOutput(elevator.getCurrentPosition() - elevator_zero_position , targetElevator);

        vertical = Range.clip(vertical, -1.0, 1.0) ;

        // Send calculated power to wheels
        elevator.setPower(vertical);

        return(Math.abs(targetElevator-(elevator.getCurrentPosition() - elevator_zero_position)) );
    }



    public void moveToPickup(){
        //going up
        elevator.setPower(-0.8);
        while (elevator.getCurrentPosition() - elevator_zero_position > 50) {

            if (readyToShoot()) {
                servoBlock.setPosition(0.3);
            } else {
                //shooterUpdate.setBackMode();
                servoBlock.setPosition(-0.5);
            }

            if (elevator.getCurrentPosition() - elevator_zero_position > 500){
                elevator.setPower(-0.4);
            }

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }


        elevator.setPower(0);

    }

}

