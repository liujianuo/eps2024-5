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

@Autonomous(name="PIDTEST")
public class Auto_PID_TEST extends LinearOpMode{
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
    protected static final double GROPEN = 0.4;
    protected static final double GRCLOSED = 0;
    protected static final double GLOPEN = 0.1;
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
    protected static final double buffertime = 0.3;

    double LFoffset = -1;
    double LBoffset = -1;
    double RFoffset = -1;
    double RBoffset = -1;

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

    public class PIDerror{
        double target;
        double time;
        double current;
        double expectedpos;
        double currenterror;
        double lasterror;
        double deriv;
        double integral;
        double buffer;

        public PIDerror(double targ, limit, buff){
            target = targ;
            time = limit;
            current = 0;
            expectedpos = -1;
            currenterror = -1;
            lasterror = 0;
            deriv = -1;
            integral = 0;
            buffer = buff;
        }

        public boolean evaluate(){
            return (Math.abs(current - target) < buffer);
        }

        public boolean update(double pos, double delta, double total){
            current = pos;
            expectedpos = Math.abs(target) < Math.abs((total/time)*target) ? target : (total/time)*target;
            currenterror = expectedpos = current;
            deriv = (currenterror - lasterror)/delta;
            integral += currenterror*delta;
            lasterror = currenterror;
        }

    }

    void setoffset(){
        LFoffset = leftfront.getCurrentPosition();
        RFoffset = rightfront.getCurrentPosition();
        LBoffset = leftback.getCurrentPosition();
        RBoffset = rightback.getCurrentPosition();
        return;
    }

    double getLF(){
        return LFoffset - leftfront.getCurrentPosition();
    }
    double getRF(){
        return RFoffset - rightfront.getCurrentPosition();
    }
    double getLB(){
        return LBoffset - leftback.getCurrentPosition();
    }
    double getRB(){
        return RBoffset - rightback.getCurrentPosition();
    }

    double calcy(){
        return (leftfront.getCurrentPosition() + rightfront.getCurrentPosition() + leftback.getCurrentPosition() + rightback.getCurrentPosition())/4;
    }
    
    double calcx(){
        return (leftfront.getCurrentPosition() - rightfront.getCurrentPosition() - leftback.getCurrentPosition() + rightback.getCurrentPosition())/4;
    }

    double calcrot(){
        return (-leftfront.getCurrentPosition() + rightfront.getCurrentPosition() - leftback.getCurrentPosition() + rightback.getCurrentPosition())/4;
    }

    public void PID(double vert, double horiz, double rota, double time){ //vert, horiz, rot, time        
        ElapsedTime PIDtime = new ElapsedTime();
        ElapsedTime CUMtime = new ElapsedTime();
        ElapsedTime CMPtime = new ElapsedTime();

        setoffset();
        PIDerror y = new PIDerror(-vert, time, 100);
        PIDerror x = new PIDerror(-horiz, time, 100);
        PIDerror rot = new PIDerror(rota, time, 0.5);

        imu.resetYaw();
        PIDtime.reset();
        CUMtime.reset();
        CMPtime.reset();
        ///*
        while(CMPtime.seconds() < buffertime){ 
            if(!y.evaluate() || !x.evaluate() || !rot.evaluate()){
                CMPtime.reset();
            }
            
            y.update(calcy, PIDtime.seconds(), CUMtime.seconds());
            x.update(calcx, PIDtime.seconds(), CUMtime.seconds());
            rot.update(calcrot, PIDtime.seconds(), CUMtime.seconds());

            double vertpower = (KP*y.currenterror) + (KD*y.deriv) + (KI*y.integral);
            double horizpower = (KP*x.currenterror) + (KD*x.deriv) + (KI*x.integral);
            double rotpower = (KPAl*rot.currenterror) + (KDAl*rot.deriv) + (KIAl*rot.integral);

            double lf = vertpower + horizpower - rotpower;
            double rb = vertpower + horizpower +  rotpower;
            double lb = vertpower - horizpower - rotpower;
            double rf = vertpower - horizpower + rotpower;

            double maximum = Double.max(1, Double.max(Double.max(Math.abs(lf), Math.abs(lb)), Double.max(Math.abs(rf), Math.abs(rb))));
            lf /= maximum;
            rf /= maximum;
            lb /= maximum;
            rb /= maximum;
            
            leftfront.setPower(lf);
            leftback.setPower(lb);
            rightfront.setPower(rf);
            rightback.setPower(rb);

            PIDtime.reset();
            
            telemetry.addData("TIME", " "+CUMtime.time() + " / " + time);
            telemetry.addData("POSX", " " + x.current + " / " + x.expectedpos + " / " + x.target);
            telemetry.addData("POSY", " " + y.current + " / " + y.expectedpos + " / " + y.target);
            telemetry.addData("ROT", " " + rot.current + " / " + rot.expectedpos + " / " + rot.target);
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

        gripR.setPosition(GRCLOSED);
        gripL.setPosition(GLCLOSED);
        imu.resetYaw();

        waitForStart(); // waiting for start;

        double leftfrontpos = leftfront.getCurrentPosition();
        double leftbackpos = leftback.getCurrentPosition();
        double rightfrontpos = rightfront.getCurrentPosition();
        double rightbackpos = rightback.getCurrentPosition();
        
        PID(2000, 2000, 0, 4);
        PID(-2000, -2000, 0, 4);
        
        //*/
    }
}
