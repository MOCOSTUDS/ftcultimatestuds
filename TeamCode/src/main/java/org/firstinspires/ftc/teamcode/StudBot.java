package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class StudBot {





    public CRServo servoLoop = null;
    private float armAngle = 0;
    public DcMotor arm = null;

    public DigitalChannel digIn1;

    public float arm_zero_position=0;
    double vertical,horizontal,pivot = 0.0;

    StudShooter shooter = new StudShooter();
    StudDrive drive = new StudDrive();
    StudIntake intake = new StudIntake();
    StudElevator elevator = new StudElevator();

    public void init(HardwareMap hardwareMap) {
        drive.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        elevator.init(hardwareMap);



        digIn1 = hardwareMap.get(DigitalChannel.class, "switch1"); //false is not pressed
        digIn1.setMode(DigitalChannel.Mode.INPUT);

        arm  = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        servoLoop  = hardwareMap.crservo.get("servo_loop");






    }

    public void my_sleep(float ms){
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }



    public void init_arm(){
        //going down

        arm.setPower(0.2);
        my_sleep(300);


        //going up
        arm.setPower(-0.2);
        do{
            my_sleep(10);
        }while(!digIn1.getState());
        arm.setPower(0);

        arm_zero_position=arm.getCurrentPosition();
    }

    public StudShooter getShooter() {
        return shooter;
    }

    public StudDrive getDrive() {
        return drive;
    }

    public StudIntake getIntake() {
        return intake;
    }

    public StudElevator getElevator() {
        return elevator;
    }

}
