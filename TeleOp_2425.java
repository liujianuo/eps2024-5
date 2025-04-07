package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="2425_basic")
public class TeleOp2425_basic extends OpMode {
// Define all motors
    DcMotor rightback;
    DcMotor leftback;
    DcMotor rightfront;
    DcMotor leftfront;
    Servo intake;
    DcMotor linear;
    Servo gripR;
    Servo gripL;
    Servo wrist;
    Servo hockey;
    
    DcMotor linact;
    
    TouchSensor front;
    CRServo triangle;
    CRServo trireverse;


    // defined constants
    protected static final double GOBILDA = 537.7; // tick/rev of gobilda and tetrix dc's
    protected static final double TETRIX = 1440;

    protected static final double XSENSITIVITY = 0.5; //Sensitivity of movement
    protected static final double YSENSITIVITY = -0.5; // (negative sign is already factored into sensitivity)
    protected static final double RSENSITIVITY = 0.5;
    protected static final double ARMSENSITIVITY = 0.7;
    protected static final double ROTORSENSITIVITY = 0.9;
    protected static final double SLIDESENSITIVITY = -1;

    protected static final double GROPEN = 0.3; //Servo positions. 
    protected static final double GRCLOSED = 0;
    protected static final double GLOPEN = 0.1;
    protected static final double GLCLOSED = 0.5;
    protected static final double INOPEN = 0;
    protected static final double INCLOSED = 0.3;
    
    protected static final double HOCKEYUP = -0.9;
    protected static final double HOCKEYDOWN = 0.88;
    
    protected static final double FUCK = 0.85;
    protected static final double WRISTUP = 0.6;
    protected static final double WRISTDOWN = 0.33;
    
    double counter = 0; // debug variable. Increments up by 1 each loop, just to verify when the loop is running
    
    double multiplier = 1; // movement speed multiplier
    static int state = 0; // state of macro
    static boolean lockout = false; // macro variable, "locks out" when macro is under way
    boolean doubleinput = false; // double input booleans to prevent multiple inputs on dual control buttons
    boolean wristreset = false;
    boolean fingerreset = false;
    boolean pinchreset = false;
    boolean hockeyreset = false; 
    int hockeypos = 2; // position trackers for dual input servos
    int wristpos = 2;
    int fingerpos = 2;
    int pinchpos = 2;

    /* // deprecated method
    public void runmotor(int position, int position2){
        armr.setTargetPosition(pos + ARMU);
        arml.setTargetPosition(pos2 + ARMU);

        armr.setPower(DEFSPEED);
        arml.setPower(DEFSPEED);

        while(opModeIsActive() && armr.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("motor1 encoder", armr.getCurrentPosition(););
            telemetry.addData("arml encoder", arml.getCurrentPosition(););
            telemetry.update();
            idle();
        }

        // set armr power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        armr.setPower(0.0);
        arml.setPower(0.0);
    }
    */
    
    public ElapsedTime timer = new ElapsedTime(); // timer for macros
    
    public void sleepmilli(long milli){ // not used i think. this sleep function "locks" the code, not allowing the code to progress until the sleep ends
        try {
            Thread.sleep(milli);
        } catch (Exception e){}
    }
    
    public void reset(){ //resets the timer at the start of each step of macro
        if(lockout == false){ // if locked out, do not reset the timer
            timer.reset();
            lockout = true;
        }
    }
    
    public void check(double time){ // checks timer, to see if macro step is complete
        if(timer.time() >= time){
            lockout = false;
            state += 1; // increment the state if the time is up
        }
    }

    public void closegrip(){ // close and open grippers. seperate function because two servos to move (for elegance)
        gripR.setPosition(GRCLOSED);
        gripL.setPosition(GLCLOSED);
    }

    public void opengrip(){
        gripR.setPosition(GROPEN);
        gripL.setPosition(GLOPEN);
    }
    
    public void init(){ // initialize function
        // Map all motors
        rightback = hardwareMap.dcMotor.get("RIGHTBACK");
        leftback = hardwareMap.dcMotor.get("LEFTBACK");
        rightfront = hardwareMap.dcMotor.get("RIGHTFRONT");
        leftfront = hardwareMap.dcMotor.get("LEFTFRONT");
        intake = hardwareMap.servo.get("PINCHER");
        linear = hardwareMap.dcMotor.get("LINSLIDE");
        
        linact = hardwareMap.dcMotor.get("ACTUATOR");
        
        gripR = hardwareMap.servo.get("GR");
        gripL = hardwareMap.servo.get("GL");
        wrist = hardwareMap.servo.get("WRIST");
        hockey = hardwareMap.servo.get("HOCKEY");
        
        front = hardwareMap.touchSensor.get("TOUCH");
        triangle = hardwareMap.crservo.get("TRIANGLE");
        trireverse = hardwareMap.crservo.get("TRIANGLER");

        // Negate direction for opposite drive motors
        leftback.setDirection(DcMotorSimple.Direction.REVERSE);
        leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightback.setDirection(DcMotorSimple.Direction.FORWARD);
        rightfront.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setDirection(DcMotorSimple.Direction.FORWARD);
        state = 0; // initialize macro state and lockout state
        lockout = false;

    }

    public void basic(){ //Basic operations that happen every loop.
        //---------------------------------//
        //   Initialisation of variables   //
        //---------------------------------//
        double x = -XSENSITIVITY*gamepad1.left_stick_x; // Variable for f/b motion
        double y = -YSENSITIVITY*gamepad1.left_stick_y; // Variable for r/l motion
        double r = RSENSITIVITY*gamepad1.right_stick_x; // Variable for rotation around center
        double slide = SLIDESENSITIVITY*gamepad2.right_stick_y; //linear slide
        double triforce = ARMSENSITIVITY*gamepad2.left_stick_y; //two bar linkage
        boolean geardown = gamepad1.left_bumper; //gear up/down multiplier
        boolean gearup = gamepad1.right_bumper;
        double rotfast = gamepad1.right_trigger; //make rotation faster
        double straightfast = gamepad1.left_trigger; //make straight movmenet faster
        
        boolean closed = gamepad2.right_bumper; // not used
        boolean open = gamepad2.left_bumper;
        
        boolean pinchswitch = gamepad2.x; // specimen pincher
        
        boolean hockeystick = gamepad1.x; 
        telemetry.addData("hockeystick", hockeypos);
        boolean fuckit = gamepad2.guide; // gripper all the way up
        
        boolean wristup = gamepad2.dpad_up; // gripper wrist up/down
        boolean wristdown = gamepad2.dpad_down;
        boolean fingerclose = gamepad2.dpad_right; // gripper fingers 
        boolean fingeropen = gamepad2.dpad_left;
        
        double actup = gamepad2.right_trigger; // linear actuator
        double actdown = gamepad2.left_trigger;
        
        double actpower = actup-actdown; // add/subtract the powers for the actuator
        
        linact.setPower(actpower); // set linear actuator power
        
        if(rotfast > 0.5){ // if rotation fast or straight fast, multiply the values up to the maximum.
            r*=100;
        }
        if(straightfast > 0.5){
            x*= 1.5;
            y*= 1.5;
        }
        
        if(wristup){ // move wrist
            wrist.setPosition(WRISTUP);
        }
        else if(wristdown){
            wrist.setPosition(WRISTDOWN);
        }
        else if(fuckit){
            wrist.setPosition(FUCK);
            wristpos = 2;
        }
        
        if(pinchswitch && !pinchreset){ // dual input prevention "pinch reset" tracks if the input was present last loop || IF button pressed AND button was not pressed last loop, we should switch the position
            if(pinchpos != 0){ // is it open (not closed)?
                closegrip(); // then close it
                pinchpos = 0;
            }
            else if(pinchpos != 1){ // is it closed (not open)? || These are set to != because we start with position "2", which is not defined
                opengrip(); // then open it
                pinchpos = 1;
            }
            pinchreset = true; // set reset to track that the button was pressed this turn
        }
        else if(!pinchswitch){ // button got un-pressed, so we reset "pinchreset"
            pinchreset = false;
        }
        
        if(hockeystick && !hockeyreset){ // same for hockey
            if(hockeypos != 0){
                hockey.setPosition(HOCKEYUP);
                hockeypos = 0;
            }
            else if(hockeypos != 1){
                hockey.setPosition(HOCKEYDOWN);
                hockeypos = 1;
            }
            hockeyreset = true;
        }
        else if(!hockeystick){
            hockeyreset = false;
        }
        
        if(fingerclose){ // deprecated method
            intake.setPosition(INCLOSED);  
        }
        else if(fingeropen){
            intake.setPosition(INOPEN);
        }

        if(gearup && !doubleinput){ // change multiplidr for gear changes
            multiplier += 0.25;
            doubleinput = true;
        }
        else if(geardown && !doubleinput){
            multiplier -= 0.25;
            doubleinput = true;
        }
        else if(!geardown && !gearup){
            doubleinput = false;
        }
        telemetry.addData("debug", doubleinput); // telemetry for debug
        telemetry.addData("multiplier", multiplier);
        x *= multiplier; // multiply inputs
        y *= multiplier;
        r *= multiplier;

        //MECANUM calculations
        double lf = r; 
        double lb = r;
        double rf = -r;
        double rb = -r;
        
        if(Math.abs((y/x)) > 4){ // if it is "close enough" to dead center up or down, only calculate f/b movement
            lf += y;
            lb += y;
            rf += y;
            rb += y;
        }
        
        else if(Math.abs((x/y)) > 4){ // if it is "close enough" to dead right/left, only calculate r/l movement
            lf += x;
            lb -= x;
            rf -= x;
            rb += x;
        }
        else{ // diagonal case
            lf += x;
            lb -= x;
            rf -= x;
            rb += x;
            lf += y;
            lb += y;
            rf += y;
            rb += y;
        }
        
        //NORMALIZE
        double maximum = Double.max(1, Double.max(Double.max(Math.abs(lf), Math.abs(lb)), Double.max(Math.abs(rf), Math.abs(rb))));
        lf /= maximum;
        rf /= maximum;
        lb /= maximum;
        rb /= maximum;

        telemetry.addData("left-front", lf); // push data to telemtry for debug
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
    
        linear.setPower(slide); // linear slide add power
        
        if(triforce > 0.2 || triforce < -0.2){ // if enough movement, then set the power, otherwise do not set power
            triangle.setPower(triforce);
            trireverse.setPower(-triforce);
        }
        else{
            triangle.setPower(0);
            trireverse.setPower(0);
        }
        
        if(front.isPressed()){ // touch sensor
            telemetry.addData("touch", "pressed");
        }
        else{
            telemetry.addData("touch", "not pressed");
        }
        telemetry.addData("linearpos", linear.getCurrentPosition()); // linear slide position
    }

    //NOW, we start the macro sections
    //each macro is its own method (function). 
    //the "state" is a two digit number. The first (tens) digit tells you which routine to run, the second (ones) digit tells you the current step of the routine.
    // a "switch" operator takes a value, and matches it to a "case" within it. Then, it runs only that piece of code (break keyword ends the rest). 
    // each "case" resets (if locked out, it doesnt reset), then it sets the power or servo positions. the "reset" state checks if it has passed the allotted time, then it moves on.
    // past the last state, we return to the null state 0
    
    public void intake(){//drop the intake. The operator may then either intake or spit out a sample BUTTON A
       
    }
    
    public void scorein(){ //take a specimen from the corner BUTTON X
        switch(state%10){
            case 1:
                //open
                reset();
                opengrip();
                check(0.5);  
                break;
            case 2: //give time for alignment
                //donothing
                if(gamepad2.x == true){
                    state += 1;
                }
                break;
            case 3:
                //close grip
                reset();
                closegrip();
                check(0.5);
                break;
            case 4:
                //up
                reset();
                linear.setPower(1);
                check(0.5);
                break;
            case 5:
                reset();
                check(0.5);
                break;
            case 6:
                //out
                reset();
                leftback.setPower(-0.5);
                leftfront.setPower(0.5);
                rightback.setPower(0.5);
                rightfront.setPower(-0.5);
                check(0.1);
                break;
            case 7:
                reset();
                check(0.5);
                break;
            case 8:
                //down
                reset();
                linear.setPower(-1);
                check(0.3);
                break;
            default:
                state = 0;
        }
    }

    public void scorehigh(){ //score on top chamber BUTTON DPAD_UP
        switch(state%10){
            case 1:
                //up
                reset();
                linear.setPower(0.8);
                check(3.5);
                break;
            case 2:
                if(gamepad2.dpad_up){
                    state += 1;
                }
                break;
            case 3:
                //open
                reset();
                leftback.setPower(-0.2);
                leftfront.setPower(0.2);
                rightback.setPower(0.2);
                rightfront.setPower(-0.2);
                check(0.5);
                break;
            case 4:
                reset();
                check(0.5);
                break;
            case 5:
                reset();
                linear.setPower(-0.8);
                check(1.3);
                break;
            case 6:
                reset();
                check(0.5);
                break;
            case 7:
                reset();
                opengrip();
                check(0.5);
                break;
            case 8:
                reset();
                leftback.setPower(-0.2);
                leftfront.setPower(0.2);
                rightback.setPower(0.2);
                rightfront.setPower(-0.2);
                check(0.5);
                break;
            default:
                state = 0;
       }
    }
    
    public void scorelow(){ //score on low chamber BUTTON DPAD_DOWN
        switch(state%10){
            case 1:
                //up
                reset();
                linear.setPower(0.8);
                check(1.5);
                break;
            case 2:
                if(gamepad2.dpad_down){
                    state += 1;
                }
                break;
            case 3:
                //open
                reset();
                leftback.setPower(-0.2);
                leftfront.setPower(0.2);
                rightback.setPower(0.2);
                rightfront.setPower(-0.2);
                check(0.5);
                break;
            case 4:
                reset();
                check(0.5);
                break;
            case 5:
                reset();
                linear.setPower(-0.8);
                check(1);
                break;
            case 6:
                reset();
                check(0.5);
                break;
            case 7:
                reset();
                opengrip();
                check(0.5);
                break;
            case 8:
                reset();
                leftback.setPower(-0.2);
                leftfront.setPower(0.2);
                rightback.setPower(0.2);
                rightfront.setPower(-0.2);
                check(0.5);
                break;
            default:
                state = 0;
       }
    }
    
    public void debugmacro(){
        switch(state%10){
            case 1:
                reset();
                telemetry.addData("debugmacro", "stage1");
                check(5);
                break;
            case 2:
                reset();
                telemetry.addData("debugmacro", "stage2");
                check(5);
                break;
            case 3:
                //down
                reset();
                if(gamepad2.guide == true){
                    state += 1;
                }
                break;
            case 4:
                reset();
                telemetry.addData("debugmacro", "stage1");
                check(5);
                break;
            case 5:
                reset();
                telemetry.addData("debugmacro", "stage1");
                check(5);
                break;
            default:
                state = 0;
       }
    }
    
    public void loop(){ // loop function
        boolean middle = false; // macro buttons, but disabled because ahmad didnt want them
        boolean corner = false;
        boolean high = false;
        boolean low = false;
        boolean debug = false;
        
        
        if(state == 0){ // if no macro is running
            counter++; // increment the counter for debug
            telemetry.addData("count", counter);
            if(middle){ // if a macro button is pressed
                state = 11; // set to the first state of said macro
            }
            if(debug){
                state = 21;
            }
            else if(corner){
                state = 31;
            }
            else if(high){
                state = 41;
            }
            else if(low){
                state = 51;
            }
        }
        
        basic(); // run all basic method (manual control)
        telemetry.addData("status", state);
        switch(state/10){ // if we are within a current state, send it to the right method (function)
            case 1:
                intake();
                telemetry.addData("macros", "intake");
                break;
            case 2:
                debugmacro();
                telemetry.addData("macros", "debug");
                break;
            case 3:
                scorein();
                telemetry.addData("macros", "score-in");
                break;
            case 4:
                scorehigh();
                telemetry.addData("macros", "score-high");
                break;
            case 5:
                scorelow();
                telemetry.addData("macros", "score-low");
                break;
            case 0:
                telemetry.addData("macros", "none");
        }
        telemetry.addData("lockout", lockout);
        telemetry.update();
    }

    public void stop(){ // do nothing in stop.

    }
}

