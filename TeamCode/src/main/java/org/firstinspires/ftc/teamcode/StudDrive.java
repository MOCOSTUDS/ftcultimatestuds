package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class StudDrive {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;


    public void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setPower(double frontLeft, double frontRight, double rearRight, double leftRear) {
        leftFrontDrive.setPower(frontLeft);
        leftRearDrive.setPower(leftRear);
        rightFrontDrive.setPower(frontRight);
        rightRearDrive.setPower(rearRight);
    }

    public void stopRobot() {
        leftFrontDrive.setPower(0.0);
        leftRearDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightRearDrive.setPower(0.0);
    }

    public void teleDrive(double  vertical, double horizontal, double pivot) {
        double rightFrontPower = -2.0*pivot + ( vertical + horizontal);
        double rightRearPower = -2.0*pivot + ( vertical - horizontal);
        double leftFrontPower = 2.0*pivot + ( vertical - horizontal);
        double leftRearPower = 2.0*pivot + ( vertical + horizontal);

        leftFrontPower    = Range.clip(leftFrontPower, -1.0, 1.0) ;
        leftRearPower    = Range.clip(leftRearPower, -1.0, 1.0) ;
        rightFrontPower    = Range.clip(rightFrontPower, -1.0, 1.0) ;
        rightRearPower    = Range.clip(rightRearPower, -1.0, 1.0) ;

        // Send calculated power to wheels
        setPower(leftFrontPower,rightFrontPower,rightRearPower,leftRearPower);
    }

}
