package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class StudShooter {

    private DcMotor elevator = null;
    private DcMotor shooterBelt = null;

    private Servo servoClaw = null;
    public Servo servoBlock = null;

    public void init(HardwareMap hardwareMap) {
        elevator  = hardwareMap.get(DcMotor.class, "elevator");
        shooterBelt  = hardwareMap.get(DcMotor.class, "shooter_belt");
        elevator.setDirection(DcMotor.Direction.FORWARD);
        shooterBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterBelt.setDirection(DcMotor.Direction.FORWARD);
        shooterBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //shooterBelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        servoClaw  = hardwareMap.servo.get("servo_claw");
        servoClaw.setPosition(0.3);

        servoBlock  = hardwareMap.servo.get("shooter_block");
        servoBlock.setPosition(-0.5);
    }

    public DcMotor getShooterMotor() {
        return shooterBelt;
    }

    public void startShooter(double velocity) {
        shooterBelt.setPower(.9);
    }

    public void stopShooter() {
        shooterBelt.setPower(0.0);
    }

    public void setClaw(double newVal) {
        servoClaw.setPosition(newVal);
    }

    public void setClawOpen() {
        servoClaw.setPosition(0.3);
    }

    public void setClawShoot() {
        servoClaw.setPosition(1.0);
    }


}
