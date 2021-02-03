package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class StudDrive {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;

    public miniPID pidX;
    public miniPID pidY;
    public miniPID pidHeading;

    public float heading_correction=0;


    /*
    public double pValueX = 0.07;
    public double iValueX = 0.00005;
    public double dValueX = -0.05;

    public double pValueY = 0.07;
    public double iValueY = 0.00005;
    public double dValueY = -0.05;

    public double pValueHeading = 0.03;
    public double iValueHeading = 0.0;
    public double dValueHeading = 0.0;
*/

    public double pValueX = 0.1;
    public double iValueX = 0.0005;
    public double dValueX = -0.05;

    public double pValueY = 0.1;
    public double iValueY = 0.0005;
    public double dValueY = -0.05;

    public double pValueHeading = 0.02;
    public double iValueHeading = 0.0005;
    public double dValueHeading = 0.0;

    public int iteration=0;

    double COUNTS_PER_INCH;

    public double targetX = 0.0;
    public  double targetHeading = 0.0;
    public double targetY = 0.0;

    public double xOffset = 0.0;
    public double yOffset = 0.0;
    public double headingOffset = 0.0;

    public double total_power = 0;

    public void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //COUNTS_PER_INCH = 307.699557*121./48.*(33./24.)*(180./72.);
        COUNTS_PER_INCH = 307.699557*121./48.*(33./24.)*(180./72.)*(19.5/48);
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

        pidX = new miniPID(pValueX, iValueX, dValueX);
        pidX.setOutputLimits(1);
        pidX.setSetpointRange(10.0);

        pidY = new miniPID(pValueY, iValueY, dValueY);
        pidY.setOutputLimits(1);
        pidY.setSetpointRange(10.0);

        pidHeading = new miniPID(pValueHeading, iValueHeading, dValueHeading);
        pidHeading.setOutputLimits(1);
        pidHeading.setSetpointRange(60.0);
    }

    public void setTarget (double xVal, double yVal, double heading) {
        targetX = xVal;
        targetY = yVal;
        targetHeading = heading;
    }

    public void setTargetfromOrgin (double xVal, double yVal, double heading) {

    double x=xVal-xOffset;
    double y=yVal-yOffset;

    double newx=
            x*Math.cos(Math.toRadians(headingOffset))
                    -y*Math.sin(Math.toRadians(headingOffset));
    double newy=
            x*Math.sin(Math.toRadians(headingOffset))
                    +y*Math.cos(Math.toRadians(headingOffset));
        targetX = newx;
        targetY = newy;
        targetHeading = heading;
}
    public void setTargetfromOrginHeading (double heading) {


        targetHeading = heading;
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

    public static double distance(double alpha, double beta) {
        double phi = Math.abs(beta - alpha) % 360;       // This is either the distance or 360 - distance
        double distance = phi;
        if (distance > 180 ) distance -=360;
        if (distance <=-180) distance +=360;
        return(distance);
    }

    public void teleDrive(double  vertical, double horizontal, double pivot) {
        double rightFrontPower = -1.0*pivot + ( vertical + horizontal);
        double rightRearPower = -1.0*pivot + ( vertical - horizontal);
        double leftFrontPower = 1.0*pivot + ( vertical - horizontal);
        double leftRearPower = 1.0*pivot + ( vertical + horizontal);

        leftFrontPower    = Range.clip(leftFrontPower, -1.0, 1.0) ;
        leftRearPower    = Range.clip(leftRearPower, -1.0, 1.0) ;
        rightFrontPower    = Range.clip(rightFrontPower, -1.0, 1.0) ;
        rightRearPower    = Range.clip(rightRearPower, -1.0, 1.0) ;

        // Send calculated power to wheels
        setPower(leftFrontPower,rightFrontPower,rightRearPower,leftRearPower);
    }



    public double move (double currentX, double currentY, double currentHeading){

        double horizontal= -pidX.getOutput(currentX , targetX);
        double vertical = pidY.getOutput(currentY , targetY);

        //double d = distance(currentHeading,targetHeading);
        //double pivot = -pidHeading.getOutput(-d, 0);
        double pivot = pidHeading.getOutput(currentHeading, targetHeading);


        double rightFrontPower = -1.0*pivot + ( vertical + horizontal);
        double rightRearPower = -1.0*pivot + ( vertical - horizontal);
        double leftFrontPower = 1.0*pivot + ( vertical - horizontal);
        double leftRearPower = 1.0*pivot + ( vertical + horizontal);


        double max=1.;

        /*

        if (Math.abs(leftFrontPower)>max) max=Math.abs(leftFrontPower);
        if (Math.abs(rightFrontPower)>max) max=Math.abs(leftFrontPower);
        if (Math.abs(leftRearPower)>max) max=Math.abs(leftFrontPower);
        if (Math.abs(rightRearPower)>max) max=Math.abs(leftFrontPower);
        setPower(leftFrontPower/max,rightFrontPower/max,rightRearPower/max,leftRearPower/max);
        */


        leftFrontPower    = Range.clip(leftFrontPower, -1.0, 1.0) ;
        leftRearPower    = Range.clip(leftRearPower, -1.0, 1.0) ;
        rightFrontPower    = Range.clip(rightFrontPower, -1.0, 1.0) ;
        rightRearPower    = Range.clip(rightRearPower, -1.0, 1.0) ;
        setPower(leftFrontPower/max,rightFrontPower/max,rightRearPower/max,leftRearPower/max);
        // Send calculated power to wheels


        return(Math.abs(targetX-currentX)+ (Math.abs(targetY-currentY)) );
    }




    public double tankMove (double speed){

        double horizontal= 0;
        double vertical = speed;
        double pivot = 0;

        double rightFrontPower = -1.0*pivot + ( vertical + horizontal);
        double rightRearPower = -1.0*pivot + ( vertical - horizontal);
        double leftFrontPower = 1.0*pivot + ( vertical - horizontal);
        double leftRearPower = 1.0*pivot + ( vertical + horizontal);


        leftFrontPower    = Range.clip(leftFrontPower, -1.0, 1.0) ;
        leftRearPower    = Range.clip(leftRearPower, -1.0, 1.0) ;
        rightFrontPower    = Range.clip(rightFrontPower, -1.0, 1.0) ;
        rightRearPower    = Range.clip(rightRearPower, -1.0, 1.0) ;

        // Send calculated power to wheels
        setPower(leftFrontPower,rightFrontPower,rightRearPower,leftRearPower);

        return(0);
    }

    public double tankMove (double speed,double pivot){

        double horizontal= 0;
        double vertical = speed;
        //double pivot = 0;

        double rightFrontPower = -1.0*pivot + ( vertical + horizontal);
        double rightRearPower = -1.0*pivot + ( vertical - horizontal);
        double leftFrontPower = 1.0*pivot + ( vertical - horizontal);
        double leftRearPower = 1.0*pivot + ( vertical + horizontal);


        leftFrontPower    = Range.clip(leftFrontPower, -1.0, 1.0) ;
        leftRearPower    = Range.clip(leftRearPower, -1.0, 1.0) ;
        rightFrontPower    = Range.clip(rightFrontPower, -1.0, 1.0) ;
        rightRearPower    = Range.clip(rightRearPower, -1.0, 1.0) ;

        // Send calculated power to wheels
        setPower(leftFrontPower,rightFrontPower,rightRearPower,leftRearPower);

        return(0);
    }




    public double pivot (double currentHeading){


        double pivot = pidHeading.getOutput(currentHeading, targetHeading);

        double rightFrontPower = -1.0*pivot ;
        double rightRearPower = -1.0*pivot ;
        double leftFrontPower = 1.0*pivot ;
        double leftRearPower = 1.0*pivot ;

        leftFrontPower    = Range.clip(leftFrontPower, -1.0, 1.0) ;
        leftRearPower    = Range.clip(leftRearPower, -1.0, 1.0) ;
        rightFrontPower    = Range.clip(rightFrontPower, -1.0, 1.0) ;
        rightRearPower    = Range.clip(rightRearPower, -1.0, 1.0) ;

        total_power = Math.abs(leftFrontPower) + Math.abs(leftRearPower)+ Math.abs(rightFrontPower)+ Math.abs(rightRearPower);
        // Send calculated power to wheels
        setPower(leftFrontPower,rightFrontPower,rightRearPower,leftRearPower);
        iteration++;

        return(Math.abs(targetHeading-currentHeading));
    }

    public void fastPID(){
        pValueX = 0.1;
        iValueX = 0.0005;
        dValueX = -0.05;

        pValueY = 0.1;
        iValueY = 0.0005;
        dValueY = -0.05;

        pValueHeading = 0.08;
        iValueHeading = 0.0005;
        dValueHeading = 0.13;

        pidX.setPID(pValueX, iValueX, dValueX);
        pidY.setPID(pValueY, iValueY, dValueY);
        pidHeading.setPID(pValueHeading, iValueHeading, dValueHeading);
    }

    public void accuratePID(){
        pValueX = 0.1;
        iValueX = 0.0005;
        dValueX = -0.05;

        pValueY = 0.1;
        iValueY = 0.0005;
        dValueY = -0.05;

        pValueHeading = 0.08;
        iValueHeading = 0.0005;
        dValueHeading = 0.13;
        pidX.setPID(pValueX, iValueX, dValueX);
        pidY.setPID(pValueY, iValueY, dValueY);
        pidHeading.setPID(pValueHeading, iValueHeading, dValueHeading);
    }

    public void accuratePIDTeleop(){
        pValueX = 0.1;
        iValueX = 0.0005;
        dValueX = -0.05;

        pValueY = 0.1;
        iValueY = 0.0005;
        dValueY = -0.05;

        pValueHeading = 0.08;
        iValueHeading = 0.0005;
        dValueHeading = 0.13;
        pidX.setPID(pValueX, iValueX, dValueX);
        pidY.setPID(pValueY, iValueY, dValueY);
        pidHeading.setPID(pValueHeading, iValueHeading, dValueHeading);
    }

    public void accuratePIDMove(){
        pValueX = 0.03;
        iValueX = 0.0005;
        dValueX = 0;

        pValueY = 0.03;
        iValueY = 0.0005;
        dValueY = 0;

        pValueHeading = 0.002;
        iValueHeading = 0.005;
        dValueHeading = 0.0;
    }

    public void bornIn (double xOff, double yOff, double headingOff){
        xOffset = xOff;
        yOffset = yOff;
        headingOffset = headingOff;

    }

    public double getCountsPerInch() {
        return COUNTS_PER_INCH;
    }


}
