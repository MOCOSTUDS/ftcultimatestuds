package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class StudIntake {
    public CRServo intakeBelt = null;
    public DcMotor intakeWheels = null;
    public CRServo servoIntake = null;

    public void init(HardwareMap hardwareMap) {
        intakeWheels  = hardwareMap.get(DcMotor.class, "intake_wheels");
        servoIntake  = hardwareMap.crservo.get("servo_intake");
        intakeBelt  = hardwareMap.crservo.get("intake_belt");

        intakeWheels.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setFeed() {
        servoIntake.setPower(-1);
        intakeWheels.setPower(1);
        intakeBelt.setPower(-1);
    }
    public void setShooterIntakesBack() {
        servoIntake.setPower(-1);
        //intakeWheels.setPower(1);
        intakeBelt.setPower(-1);
    }

    public void setBack() {
        servoIntake.setPower(1);
        intakeWheels.setPower(-0.5);
        intakeBelt.setPower(1);
    }
    public void setFrontIntakeBack() {
        //servoIntake.setPower(1);
        intakeWheels.setPower(-1);
        //intakeBelt.setPower(1);
    }

    public void stopAll() {
        servoIntake.setPower(0);
        intakeBelt.setPower(0);
        intakeWheels.setPower(0);
    }
    public void setBeltIntakeBack() {
        //servoIntake.setPower(1);
        //intakeWheels.setPower(-1);
        intakeBelt.setPower(1);
    }


}
