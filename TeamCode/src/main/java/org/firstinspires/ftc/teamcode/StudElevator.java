package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StudElevator {

    public DcMotor elevator = null;
    public DigitalChannel digIn0;

    public float elevator_zero_position=0;

    public void init(HardwareMap hardwareMap) {
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator.setDirection(DcMotor.Direction.FORWARD);
        digIn0 = hardwareMap.get(DigitalChannel.class, "switch0"); //false is not pressed
        digIn0.setMode(DigitalChannel.Mode.INPUT);
        elevator_zero_position = elevator.getCurrentPosition();
        findZeroPoint();
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

}

