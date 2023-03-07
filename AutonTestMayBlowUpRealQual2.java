package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class AutonTestMayBlowUpRealQual2 extends LinearOpMode {
  private DcMotor frontr;
  private DcMotor frontl;
  private DcMotor bottomr;
  private DcMotor bottoml;
  private DcMotor shooter;
  private CRServo frontservo;
  private CRServo lowestServo;
  private CRServo middleServo;
  private CRServo highestServo;
  private DistanceSensor distanceSensor;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
   
//Basic algorithms to move the robot
  private void forward(float power) throws InterruptedException {
    frontr.setPower(power);
    frontl.setPower(power);
    bottomr.setPower(power);
    bottoml.setPower(power);
  
  }
  private void servo(float power) throws InterruptedException {
    lowestServo.setPower(power);
    middleServo.setPower(power);
    highestServo.setPower(power);
  }
  
  private void sideways(float power) throws InterruptedException {
     frontr.setPower(power);
     frontl.setPower(-power);
     bottomr.setPower(-power);
     bottoml.setPower(power);
  }
   
   
  @Override
  public void runOpMode() throws InterruptedException {
    frontr = hardwareMap.dcMotor.get("frontr");
    frontl = hardwareMap.dcMotor.get("frontl");
    bottomr = hardwareMap.dcMotor.get("bottomr");
    bottoml = hardwareMap.dcMotor.get("bottoml");
    shooter = hardwareMap.dcMotor.get("shooter");
    distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
    lowestServo = hardwareMap.get(CRServo.class, "lowestServo"); 
    middleServo = hardwareMap.get(CRServo.class, "middleServo");
    highestServo = hardwareMap.get(CRServo.class, "highestServo");
    frontservo = hardwareMap.get(CRServo.class, "frontservo");

    // Reverse one of the drive motors.
    frontr.setDirection(DcMotorSimple.Direction.FORWARD);
    frontl.setDirection(DcMotorSimple.Direction.REVERSE);
    bottomr.setDirection(DcMotorSimple.Direction.FORWARD);
    bottoml.setDirection(DcMotorSimple.Direction.REVERSE);
    
    
    
    
    
    
    
    shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
  
    waitForStart();
    //shoot cus shooter slow :(
      shooter.setPower(.54);
    //Right
      sideways(100);
    //shoot one String
      servo(100);
      sleep(50);
    //shoot another RING 
      servo(100);
      sleep(50);
    //shoot another RING  servo(100);
      servo(100);
      sleep(50);
    //more right
      //sideways(100);
    //move forward
      //forward(100);
    //move left 
      //sideways(-100);
    //scan rings distance
      //sensorRange.getDistance(DistanceUnit.CM);
    //Constant rotaton
    
      if (distanceSensor.getDistance(DistanceUnit.CM) >11.6) {
        //0 RINGS 
        sideways(100);
        forward(-100);
        
      }
      else if (distanceSensor.getDistance(DistanceUnit.CM) < 5.5) {
        //4 RINGS, also pickup rings?!?!?
        sideways(100);
        forward(-100);
        
        
      }
      else {
        
      }
    
      shooter.setTargetPosition(1000000000); //Set the motor to an insanely high position to try and keep it constantly running
    
    
      telemetry.addData("distance", String.format("%.01f", distanceSensor.getDistance(DistanceUnit.CM)));
      telemetry.update();
      //>>>>>>>>>>>>>>>>>>>>>Colour code can be found in AutonTestMayBlowUp.java 
      
    // todo: write your code here
  }
}


