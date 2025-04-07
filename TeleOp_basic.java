package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="debug")
public class TeleOp2425_basic extends OpMode {
// Define all motors
    DcMotor rightback;
    DcMotor leftback;
    DcMotor rightfront;
    DcMotor leftfront;


    // defined constants
    protected static final double GOBILDA = 537.7; // tick/rev of gobilda and tetrix dc's
    protected static final double TETRIX = 1440;
    protected static final double XSENSITIVITY = 0.5;
    protected static final double YSENSITIVITY = -0.5;
    protected static final double RSENSITIVITY = 0.5;  
    
    public void init(){
        // Map all motors
        rightback = hardwareMap.dcMotor.get("RIGHTBACK");
        leftback = hardwareMap.dcMotor.get("LEFTBACK");
        rightfront = hardwareMap.dcMotor.get("RIGHTFRONT");
        leftfront = hardwareMap.dcMotor.get("LEFTFRONT");

        // Negate direction for opposite drive motors
        leftback.setDirection(DcMotorSimple.Direction.REVERSE);
        leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightback.setDirection(DcMotorSimple.Direction.FORWARD);
        rightfront.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    
    public void loop(){
        double x = -XSENSITIVITY*gamepad1.left_stick_x; // Variable for f/b motion
        double y = -YSENSITIVITY*gamepad1.left_stick_y; // Variable for r/l motion
        double r = RSENSITIVITY*gamepad1.right_stick_x; // Variable for rotation around center

        double lf = r; 
        double lb = r;
        double rf = -r;
        double rb = -r;
        
        if(Math.abs((y/x)) > 0.5){
            lf += y;
            lb += y;
            rf += y;
            rb += y;
        }
        
        if(Math.abs((x/y)) > 0.5){
            lf += x;
            lb -= x;
            rf -= x;
            rb += x;
        }
        
        //NORMALIZE
        double maximum = Double.max(1, Double.max(Double.max(Math.abs(lf), Math.abs(lb)), Double.max(Math.abs(rf), Math.abs(rb))));
        lf /= maximum;
        rf /= maximum;
        lb /= maximum;
        rb /= maximum;

        telemetry.addData("left-front", lf);
        telemetry.addData("right-front", rf);
        telemetry.addData("left-back", lb);
        telemetry.addData("right-back", rb);

        telemetry.addData("left-front POS", leftfront.getCurrentPosition());
        telemetry.addData("right-front POS", rightfront.getCurrentPosition());
        telemetry.addData("left-back POS", leftback.getCurrentPosition());
        telemetry.addData("right-back POS", rightback.getCurrentPosition());

        leftback.setPower(lb);
        leftfront.setPower(lf);
        rightback.setPower(rb);
        rightfront.setPower(rf);

        telemetry.update();
    }

    public void stop(){

    }
}

