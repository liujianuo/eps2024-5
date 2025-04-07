import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;  

import java.text.DecimalFormat;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="40+3F3")
public class Auto_2_3_1 extends LinearOpMode{
    // Define all motors
    DcMotor rightback;
    DcMotor leftback;
    DcMotor rightfront;
    DcMotor leftfront;
    Servo intake;
    DcMotor linear;
    Servo gripR;
    Servo gripL;
    TouchSensor front;
    CRServo triangle;
    CRServo trireverse;
    IMU imu;

    // defined constants
    protected static final double GOBILDA = 537.7; // tick/rev of gobilda and tetrix dc's
    protected static final double TETRIX = 1440;
    protected static final double XSENSITIVITY = 0.5;
    protected static final double YSENSITIVITY = -0.5;
    protected static final double RSENSITIVITY = 0.5;
    protected static final double ARMSENSITIVITY = 0.5;
    protected static final double ROTORSENSITIVITY = 0.9;
    protected static final double SLIDESENSITIVITY = -0.7;
    protected static final double GROPEN = 0.43;
    protected static final double GRCLOSED = 0;
    protected static final double GLOPEN = 0.07;
    protected static final double GLCLOSED = 0.5;
    protected static final double KP = 0.01;
    protected static final double KD = 0;
    protected static final double KI = 0.001;
    
    protected static final double KPAl = 0.1;
    protected static final double KDAl = 0;
    protected static final double KIAl = 0.0001;

    protected static final double KPA = 0.1;
    protected static final double KDA = 0.0005;
    protected static final double KIA = 0.0001;
    protected static final double buffertime = 0.15;

    double setangle;

    static int state = 0;
    static boolean lockout = false;
    boolean doubleinput = false;

    //------------------------------------------------//
    //This Function is to apply drive values to motors//
    //------------------------------------------------//
    
    public ElapsedTime timer = new ElapsedTime();

    public void pause(double time, String msg){
        timer.reset();
        while(timer.time() < time && opModeIsActive()){
            telemetry.addData("waiting", " "+timer.time() + " / " + time);
            telemetry.addData("action", msg);
            telemetry.update();
        }
    }

    /*
    public void sleepmilli(long milli){
        try {
            Thread.sleep(milli);
        } catch (Exception e){}
    }
    */

    public void closegrip(){
        gripR.setPosition(GRCLOSED);
        gripL.setPosition(GLCLOSED);
    }

    public void opengrip(){
        gripR.setPosition(GROPEN);
        gripL.setPosition(GLOPEN);
    }

    public double getangle(){
        double raw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if(raw <= -90 && raw >= -180){
            return (raw + 360);
        }
        else return raw;
    }

    public void fbPID(double ticks, double time){ //time in seconds
        double target = -ticks;
        double targettime = time;
        double lasterror = 0;
        double lastangle = 0;
        ElapsedTime PIDtime = new ElapsedTime();
        ElapsedTime CUMtime = new ElapsedTime();
        ElapsedTime CMPtime = new ElapsedTime();
        double referencepos = leftfront.getCurrentPosition();
        double position = 0;
        
        double I = 0;
        double Ia = 0;
        
        PIDtime.reset();
        CUMtime.reset();
        CMPtime.reset();
        ///*
        while(CMPtime.seconds() < buffertime && CUMtime.seconds() < time*3){ 
            if(Math.abs(position - target) > 100 || Math.abs(setangle - getangle()) > 0.5){
                CMPtime.reset();
            }
            double expectedpos = Math.abs(target) < Math.abs((CUMtime.seconds()/targettime)*target) ? target : (CUMtime.seconds()/targettime)*target;
            double delta = PIDtime.seconds();
            position = leftfront.getCurrentPosition() - referencepos;
            
            double error = expectedpos - position;
            double dx = (error-lasterror)/delta;
            I += error*delta;

            double Yaw = getangle();
            double roterror = Yaw;
            double da = (roterror - lastangle)/delta;
            Ia += roterror*delta;

            double power = (KP*error) + (KD*dx) + (KI*I);

            double rot = (KPAl*roterror) + (KDAl*da) + (KIAl*Ia);

            double lf = power - rot;
            double rb = power + rot;
            double lb = power - rot;
            double rf = power + rot;

            double maximum = Double.max(1, Double.max(Double.max(Math.abs(lf), Math.abs(lb)), Double.max(Math.abs(rf), Math.abs(rb))));
            lf /= maximum;
            rf /= maximum;
            lb /= maximum;
            rb /= maximum;
            
            leftfront.setPower(lf);
            leftback.setPower(lb);
            rightfront.setPower(rf);
            rightback.setPower(rb);

            lasterror = error;
            lastangle = roterror;
            PIDtime.reset();
            
            telemetry.addData("TIME", " "+CUMtime.time() + " / " + targettime);
            telemetry.addData("BUFF", " " + CMPtime.time());
            telemetry.addData("POS", " " + position + " / " + expectedpos + " / " + target);
            telemetry.addData("power", power);
            telemetry.addData("rot", rot);
            telemetry.addData("error",error);
            telemetry.addData("yaw", Yaw);
            telemetry.update();
        }
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0);
        rightback.setPower(0);
    }

    public void rlPID(double ticks, double time){ //time in seconds
        double target = -ticks;
        double targettime = time;
        double lasterror = 0;
        double lastangle = 0;
        ElapsedTime PIDtime = new ElapsedTime();
        ElapsedTime CUMtime = new ElapsedTime();
        ElapsedTime CMPtime = new ElapsedTime();
        double referencepos = leftfront.getCurrentPosition();
        double position = leftfront.getCurrentPosition() - referencepos;
        
        double I = 0;
        double Ia = 0;
        
        PIDtime.reset();
        CUMtime.reset();
        CMPtime.reset();
        ///*
        while(CMPtime.seconds() < buffertime && CUMtime.seconds() < time*3){ 
            if(Math.abs(position - target) > 100 || Math.abs(setangle - getangle()) > 0.5){
                CMPtime.reset();
            }
            double expectedpos = Math.abs(target) < Math.abs((CUMtime.seconds()/targettime)*target) ? target : (CUMtime.seconds()/targettime)*target;
            double delta = PIDtime.seconds();
            position = leftfront.getCurrentPosition() - referencepos;
            
            double error = expectedpos - position;
            double dx = (error-lasterror)/delta;
            I += error*delta;

            double Yaw = getangle();
            double roterror = Yaw;
            double da = (roterror - lastangle)/delta;
            Ia += roterror*delta;

            double power = (KP*error) + (KD*dx) + (KI*I);

            double rot = (KPAl*roterror) + (KDAl*da) + (KIAl*Ia);

            double lf = power - rot;
            double rb = power + rot;
            double lb = -power - rot;
            double rf = -power + rot;

            double maximum = Double.max(1, Double.max(Double.max(Math.abs(lf), Math.abs(lb)), Double.max(Math.abs(rf), Math.abs(rb))));
            lf /= maximum;
            rf /= maximum;
            lb /= maximum;
            rb /= maximum;
            
            leftfront.setPower(lf);
            leftback.setPower(lb);
            rightfront.setPower(rf);
            rightback.setPower(rb);

            lasterror = error;
            lastangle = roterror;
            PIDtime.reset();
            
            telemetry.addData("TIME", " "+CUMtime.time() + " / " + targettime);
            telemetry.addData("BUFF", " " + CMPtime.time());
            telemetry.addData("POS", " " + position + " / " + expectedpos + " / " + target);
            telemetry.addData("power", power);
            telemetry.addData("rot", rot);
            telemetry.addData("error",error);
            telemetry.addData("yaw", Yaw);
            telemetry.update();
        }
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0);
        rightback.setPower(0);
    }

    public void rotPID(double degrees, double time){ //time in seconds, angle between -90 and 270, does not cross W 
        imu.resetYaw();
        double target = degrees;
        double targettime = time;
        double lastangle = 0;
        ElapsedTime PIDtime = new ElapsedTime();
        ElapsedTime CUMtime = new ElapsedTime();
        ElapsedTime CMPtime = new ElapsedTime();
        double curangle = 0;
        
        double Ia = 0;
        
        PIDtime.reset();
        CUMtime.reset();
        CMPtime.reset();
        ///*
        while(CMPtime.seconds() < buffertime){ 
            if(Math.abs(curangle - target) > 0.5){
                CMPtime.reset();
            }
            curangle = getangle();
            double expectedang = Math.abs(target) < Math.abs((CUMtime.seconds()/targettime)*target) ? target : (CUMtime.seconds()/targettime)*target;
            double delta = PIDtime.seconds();

            double roterror = expectedang - curangle;
            double da = (roterror - lastangle)/delta;
            Ia += roterror*delta;

            double rot = (KPA*roterror) + (KDA*da) + (KIA*Ia);

            double lf = rot;
            double rb = -rot;
            double lb = rot;
            double rf = -rot;

            double maximum = Double.max(1, Double.max(Double.max(Math.abs(lf), Math.abs(lb)), Double.max(Math.abs(rf), Math.abs(rb))));
            lf /= maximum;
            rf /= maximum;
            lb /= maximum;
            rb /= maximum;
            
            leftfront.setPower(lf);
            leftback.setPower(lb);
            rightfront.setPower(rf);
            rightback.setPower(rb);

            lastangle = roterror;
            PIDtime.reset();
            
            telemetry.addData("TIME", " " + CUMtime.time() + " / " + targettime);
            telemetry.addData("BUFF", " " + CMPtime.time());
            telemetry.addData("power", lf);
            telemetry.addData("yaw", curangle + " / " + expectedang + " / " + target);
            telemetry.update();
        }
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0);
        rightback.setPower(0);
        imu.resetYaw();
    }

    public void runOpMode(){
        // Map all motors
        rightback = hardwareMap.dcMotor.get("RIGHTBACK");
        leftback = hardwareMap.dcMotor.get("LEFTBACK");
        rightfront = hardwareMap.dcMotor.get("RIGHTFRONT");
        leftfront = hardwareMap.dcMotor.get("LEFTFRONT");
        intake = hardwareMap.servo.get("PINCHER");
        linear = hardwareMap.dcMotor.get("LINSLIDE");
        gripR = hardwareMap.servo.get("GR");
        gripL = hardwareMap.servo.get("GL");
        front = hardwareMap.touchSensor.get("TOUCH");
        triangle = hardwareMap.crservo.get("TRIANGLE");
        trireverse = hardwareMap.crservo.get("TRIANGLER");
        
        imu = hardwareMap.get(IMU.class, "imu");

        // Negate direction for opposite drive motors
        leftback.setDirection(DcMotorSimple.Direction.FORWARD);
        leftfront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightback.setDirection(DcMotorSimple.Direction.REVERSE);
        rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
        linear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Initialise arm motors
        //gripper.setDirection(Servo.Direction.FORWARD);
        // Define Motor Values

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        
        telemetry.addData("Mode", "calibrating");
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        setangle = 0.0;

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
        
        imu.resetYaw();

        waitForStart(); // waiting for start;

        double leftfrontpos = leftfront.getCurrentPosition();
        double leftbackpos = leftback.getCurrentPosition();
        double rightfrontpos = rightfront.getCurrentPosition();
        double rightbackpos = rightback.getCurrentPosition();
        
        gripR.setPosition(GRCLOSED);
        gripL.setPosition(GLCLOSED);
        pause(0.15, "buffer");
        
        linear.setPower(0.95);
        fbPID(1400, 2); // approach sub
        linear.setPower(0);
        
        linear.setPower(-1);
        pause(0.45, "score"); //score
        linear.setPower(0);
        
        gripR.setPosition(GROPEN);
        gripL.setPosition(GLOPEN);
        pause(0.2, "buffer");
        
        triangle.setPower(0.1);
        trireverse.setPower(-0.1);
        fbPID(-500, 0.1); //back up
        
        linear.setPower(-1);
        rlPID(1400, 0.4); //line up 
        
        

        fbPID(2000, 0.7);
        linear.setPower(0);

        rlPID(600, 0.3);

        
        fbPID(-2200, 0.6);//pushfirst
        
        fbPID(2200, 0.5); //second
        rlPID(750, 0.25);
        fbPID(-2200, 0.5);
        
        fbPID(2200, 0.5); //third
        rlPID(800, 0.3);
        fbPID(-2200, 0.5);
        
        fbPID(700, 0.5);
        
        rlPID(-1100, 0.4);
        
        rotPID(180, 0.6); //turn around

        
        fbPID(950, 1);
        
        gripR.setPosition(GRCLOSED);
        gripL.setPosition(GLCLOSED);
        pause(0.15, "buffer");

        linear.setPower(1);
        pause(0.15, "buffer");
        linear.setPower(0);
        fbPID(-300, 0.2); //grab
        linear.setPower(-1);
        linear.setPower(0);
        
        rotPID(179, 0.6); //turn around
        linear.setPower(0.69);
        rlPID(-2300, 0.5);

        
        
        fbPID(1250, 1.1); // approach sub
        linear.setPower(0);
        
        linear.setPower(-1);
        pause(0.5, "score"); //score
        linear.setPower(0);
        
        gripR.setPosition(GROPEN);
        gripL.setPosition(GLOPEN);
        pause(0.4, "buffer");
        
        triangle.setPower(-0.2);
        trireverse.setPower(0.2);
        
        linear.setPower(-0.90);
        fbPID(-1150, 1);
        linear.setPower(0);
        
        
        
        rlPID(2500, 1);
        
        rotPID(180, 0.6);
        
        fbPID(900, 1);
        
        gripR.setPosition(GRCLOSED);
        gripL.setPosition(GLCLOSED);
        pause(0.2, "buffer");
        
        
        //*/
    }
}
