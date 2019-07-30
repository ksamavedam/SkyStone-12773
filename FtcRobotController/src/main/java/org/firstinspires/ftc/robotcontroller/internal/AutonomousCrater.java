package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import java.util.Arrays;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;
import java.lang.*;
// Code for Auto on Crater Side
@Autonomous(name="AutonomousCrater", group="Linear Opmode")

public class AutonomousCrater extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AQapiaj/////AAABmR9MFwGXtUXlokDNoVqBfPgVJQUtQjEGM5ThSHmsuy4picaSUk8W+xn1vM+EV1DbJfrr58EOVEJdMfLFvG4An8oN8YDvHB44IGFPAmQBdHv3RkhbYEgWU/guwcEjwIXtcRTRt/J0PmTZG2xnyDxFfAk+AOUVtLE/Ze481z/We0oTolHGpStuPrUhGKGQcY7noVFb2q2LU/3DRoUKg7R1P7y93lluKthHr2BXZSFuHqN4CmEzXdeJKQ+vZrJxorguqiIdyiNvJ1fjXAtCATQeLAOwGQWp7HlRb/T9UUf/KXLcYxKKQV3NPtvE3iPuRkDceF3IvbPbX3i32+MKXZqKFHhpZwomR9PpqEzaxgG8pJ1I";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    RobotHardware rh;
    MecanumMath mwm;
    @Override
    public void runOpMode(){
        rh = new RobotHardware("GrizzlyPear", hardwareMap);
        mwm = new MecanumMath(rh);
        telemetry.addData("Before IMU Init","");
        telemetry.update();
        
        if(!rh.imu.isGyroCalibrated()){
                    this.sleep(50);
        }
        
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        
        resetRunMode();

        rh.latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int encoders;
            double Power1;
            double Power2;
            double Power3;
            double Power4;
            double position = 5;

            //Lower Robot
            //Detect Gold Mineral
    /*
            while (opModeIsActive() && position == 5) {
                position = sampling(); // -1 : Left, 0: Center ; 1: Right
                position = 0;
            }
    */            //lowerPear();
      
            position = 10;      
            shakeyshakey();

            //Move to gold detection

            //Run over gold mineral
            //Left gold
            if (position == -1) {
                //move towards gold
                encoders = horizontalEncoder(16);
                move(270, encoders, 1);
                resetRunMode();
                
                encoders = verticalEncoder(16);
                move(0, encoders, 0);
                resetRunMode();
                
                //nudge the gold
                encoders = horizontalEncoder(13);
                move(270, encoders, 1);
                resetRunMode();
                
                //go back
                encoders = horizontalEncoder(10);
                move(90, encoders, 1);
                resetRunMode();
                
                //move forward
                encoders = verticalEncoder(26);
                move(0, encoders, 0);
                resetRunMode();
            }
            else if (position == 10) {
                claimDepot();
/*                // Go towards wall  - forward
                encoders = horizontalEncoder(24);
                move(270, encoders, 0);
                resetRunMode();
 */
                break;
            }
            //Center gold
            else if (position == 0) {
                // nudge the gold
                encoders = horizontalEncoder(30);
                move(270, encoders, 0);
                resetRunMode();

                // Go back
                encoders = horizontalEncoder(10);
                move(90, encoders, 1);
                resetRunMode();

                // Go towards wall  - forward
                encoders = verticalEncoder(43);
                move(0, encoders, 0);
                resetRunMode();
                telemetry.addData("Status", "TURN 40");
                telemetry.update();
            }
            //Right gold
            else {
                //move towards gold
                encoders = horizontalEncoder(16);
                move(270, encoders, 2);
                resetRunMode();

                encoders = verticalEncoder(20);
                move(180, encoders, 0);
                resetRunMode();
                
                //nudge gold
                encoders = horizontalEncoder(13);
                move(270, encoders, 1);
                resetRunMode();
                
                //go back
                encoders = horizontalEncoder(10);
                move(90, encoders, 1);
                resetRunMode();
                
                //Go towards wall
                encoders = verticalEncoder(60);
                move(0, encoders, 0);
                resetRunMode();
            }

            // Common
            rotate(118);
            resetRunMode();
            telemetry.addData("Status", "Vert 16");
            telemetry.update();

            encoders = verticalEncoder(32);
            move(180, encoders, 0);
            resetRunMode();

            telemetry.addData("Status", "HZ 40");
            telemetry.update();

            // Claim Depot
            telemetry.addData("Status", "Claim");
            telemetry.update();

            claimDepot();
            telemetry.addData("Status", "Park");
            telemetry.update();

            // Park
            encoders = verticalEncoder(53);
            move(0, encoders, 0);
            resetRunMode();



            //Extend arm into crater
            //ParkArm();
            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            break;
        }
    }
    public void lowerPear(){
        rh.latchMotor.setTargetPosition(-9450);
        rh.latchMotor.setPower(-1);
        while (opModeIsActive() && rh.latchMotor.isBusy()){
            telemetry.addData("Encoders", rh.latchMotor.getCurrentPosition());
            telemetry.update();
        }
        rh.latchMotor.setPower(0);
    }

    public void resetRunMode() {
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void shakeyshakey(){
        int encoders = verticalEncoder(.5);
        move(0, encoders, 0);
        encoders = verticalEncoder(.5);
        move(180, encoders, 0);
    }
    public int sampling(){
        int position = -1;
        int count = 0 ;
        if (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        Recognition lowest = null;
                        Recognition secondLowest = null;
                        for (Recognition recognition : updatedRecognitions) {
                            if (lowest == null || recognition.getRight() < lowest.getRight()) {
                                secondLowest = lowest;
                                lowest = recognition;
                            } else if (secondLowest == null || recognition.getRight() < secondLowest.getRight()) {
                                secondLowest = recognition;
                            }
                        }
                        if (secondLowest == null) {
                            continue;
                        }
                        List<Recognition> lowests = Arrays.asList(lowest, secondLowest);
                        for (Recognition recognition : lowests) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getTop();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getTop();
                            }
                        }
                        if (goldMineralX == -1) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            return 1;
                        } else if (goldMineralX > silverMineral1X) {
                            telemetry.addData("Gold Mineral Position", "Center");
                            return 0;
                        } else {
                            telemetry.addData("Gold Mineral Position", "Left");
                            return -1;
                        }
                    }
                    telemetry.update();
                }
            
                while(count++ > 10000)
                    return 0; // Return center if we failed to detect
            }
            if (tfod != null) {
                tfod.shutdown();
            }
        }
        return 5;  // will never happen
    }
    private void initVuforia() {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = rh.hwMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", rh.hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public int verticalEncoder(double distance) {
        int encoders;
        double encPerInch = rh.encPerInchV;
        encoders = (int) Math.round(distance*encPerInch);
        telemetry.addData("verticalEncoder: encPerInch:",encPerInch);
        telemetry.addData("verticalEncoder",encoders);
        telemetry.update();

        return encoders;
    }
    public int horizontalEncoder(double distance) {
        int encoders;
        double encPerInch = rh.encPerInchH;
        encoders = (int) Math.round(distance*encPerInch);
        telemetry.addData("HzEncoder: encPerInch:",encPerInch);
        telemetry.addData("HzEncoder",encoders);
        telemetry.update();
        return encoders;
    }
    public int rotationEncoder(double degrees) {
        int encoders;
        double encPerInch = rh.encPerInchR;
        encoders = (int) Math.round(degrees*encPerInch);
        return encoders;
    }
    public int diagonalEncoder(double distance) {
        int encoders;
        double encPerInch = rh.encPerInchD;
        encoders = (int) Math.round(distance*encPerInch);
        return encoders;
    }
    public void move(double direction, int encoders, int side) {
        double Power1 = 0;
        double Power2 = 0;
        double Power3 = 0;
        double Power4 = 0;
        double speed = 0;
        direction = Math.toRadians(direction);

        if (side == 0) {
            speed = 0.5;
        }
        else if (side == 1) {
            speed = .4;
        }
        else if (side == 2)
            speed = 0.8;
            
        Power1 = mwm.power1(.4, 0, direction);
        Power2 = mwm.power2(.4, 0, direction);
        Power3 = mwm.power3(.4, 0, direction);
        Power4 = mwm.power4(.4, 0, direction);
    
        // Send calculated power to wheels
        rh.motor1.setPower(Power1);
        rh.motor2.setPower(Power2);
        rh.motor3.setPower(Power3);
        rh.motor4.setPower(Power4);
        if (Power1 > 0) {
            rh.motor1.setTargetPosition(encoders);
        }
        else {
            rh.motor1.setTargetPosition(-encoders);
        }
        if (Power2 > 0) {
            rh.motor2.setTargetPosition(encoders);
        }
        else {
            rh.motor2.setTargetPosition(-encoders);
        }
        if (Power3 > 0) {
            rh.motor3.setTargetPosition(encoders);
        }
        else {
            rh.motor3.setTargetPosition(-encoders);
        }
        if (Power4 > 0) {
            rh.motor4.setTargetPosition(encoders);
        }
        else {
            rh.motor4.setTargetPosition(-encoders);
        }
        while (opModeIsActive() && rh.motor1.isBusy()){
            telemetry.addData("encoders", encoders);
            telemetry.addData("power1", Power1);
            telemetry.addData("power2", Power2);
            telemetry.addData("power3", Power3);
            telemetry.addData("power4", Power4);
            telemetry.addData("status", "moving");
            telemetry.update();

        }
        rh.motor1.setPower(0);
        rh.motor2.setPower(0);
        rh.motor3.setPower(0);
        rh.motor4.setPower(0);

            telemetry.addData("encoders", encoders);
            telemetry.addData("encoders1", rh.motor1.getTargetPosition());
            telemetry.addData("encoders2", rh.motor2.getTargetPosition());
            telemetry.addData("encoders3", rh.motor3.getTargetPosition());
            telemetry.addData("encoders4", rh.motor4.getTargetPosition());
            telemetry.update();
            this.sleep(500);
    }
    public void rotate(double degrees) {
        double Power1;
        double Power2;
        double Power3;
        double Power4;
        double angleAtStart = rh.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angleTurned = 0;
        double correctionAngle = 6; // For power of 0.3
        double basePower = 0.3;

        rh.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rh.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rh.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rh.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rh.motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rh.motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rh.motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rh.motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(degrees > 0) {
            Power1 = basePower ;
            Power2 = -basePower ;
            Power3 = -basePower ;
            Power4 = basePower;
            degrees -= correctionAngle;
        }
        else {
            Power1 = -basePower ;
            Power2 = basePower ;
            Power3 = basePower ;
            Power4 = -basePower;
            degrees += correctionAngle;
        }

        rh.motor1.setPower(Power1);
        rh.motor2.setPower(Power2);
        rh.motor3.setPower(Power3);
        rh.motor4.setPower(Power4);



        while (opModeIsActive()&& Math.abs(angleTurned) < Math.abs(degrees)) {

            angleTurned = rh.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - angleAtStart;

            telemetry.addData("power1", Power1);
            telemetry.addData("power2", Power2);
            telemetry.addData("power3", Power3);
            telemetry.addData("power4", Power4);
            telemetry.addData("angleAtStart", angleAtStart);
            telemetry.addData("anglesFinal", degrees);
            telemetry.addData("anglesCurrent", angleTurned);
            telemetry.addData("status", "rotating");
            telemetry.update();
        }

        rh.motor1.setPower(0);
        rh.motor2.setPower(0);
        rh.motor3.setPower(0);
        rh.motor4.setPower(0);

    }
    public void claimDepot(){
        telemetry.addData("status", "Depot");
        telemetry.update();
        rh.depot.setPosition(0.5);
        sleep(2000);
    }
    public void ParkArm(){
        rh.extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.extendMotor.setTargetPosition(-1500);
        double offset = rh.extendMotor.getCurrentPosition();
        rh.extendMotor.setPower(.7);
        while (opModeIsActive() && rh.extendMotor.getCurrentPosition()-offset > -1500) {
            telemetry.addData("Encoders", rh.extendMotor.getCurrentPosition()-offset);
            telemetry.update();
        }
        rh.extendMotor.setPower(0);

        rh.dumpMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.dumpMotor.setTargetPosition(2000);
        rh.dumpMotor.setPower(.8);
        //offset = rh.dumpMotor.getCurrentPosition();
        //double power = .2;
        //while (opModeIsActive() && rh.dumpMotor.getCurrentPosition()-offset < 1000) {
           // rh.dumpMotor.setPower(power);
            //if (Math.abs(power) < .4) {
              //  power = power +.01;
            //}
            telemetry.addData("DumpEncoders", rh.dumpMotor.getCurrentPosition()-offset);
            telemetry.update();
        //}
        rh.dumpMotor.setPower(0);
    }
}
