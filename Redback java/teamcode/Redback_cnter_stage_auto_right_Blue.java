package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Redback_cnter_stage_auto_right_Blue")
public class Redback_cnter_stage_auto_right_Blue extends LinearOpMode {

  private DcMotor left_back;
  private DcMotor left_front;
  private DcMotor right_back;
  private DcMotor right_front;
  private DcMotor arm_1;		  
  private Servo left_pixl_hand;
  private Servo right_pixl_hand;
  private Servo servo_arm;

  @Override
  public void runOpMode() {
	left_back = hardwareMap.get(DcMotor.class, "left_back");
	left_front = hardwareMap.get(DcMotor.class, "left_front");
	right_back = hardwareMap.get(DcMotor.class, "right_back");
	right_front = hardwareMap.get(DcMotor.class, "right_front");
	arm_1 = hardwareMap.get(DcMotor.class, "arm_1");
	right_pixl_hand = hardwareMap.get(Servo.class, "right_pixl_hand");
	left_pixl_hand = hardwareMap.get(Servo.class, "left_pixl_hand");
	servo_arm = hardwareMap.get(Servo.class, "servo_arm");

		right_back.setDirection(DcMotor.Direction.REVERSE);
		right_front.setDirection(DcMotor.Direction.REVERSE);
		left_pixl_hand.setDirection(Servo.Direction.REVERSE);
		right_pixl_hand.setDirection(Servo.Direction.FORWARD);
		servo_arm.setDirection(Servo.Direction.REVERSE);
		arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		arm_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		arm_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		left_pixl_hand.setPosition(0.3);
		right_pixl_hand.setPosition(0.8);
		servo_arm.setPosition(0);
		
	waitForStart();
	if (opModeIsActive()) {
		servo_arm.setPosition(0.56);
	  left_back.setPower(-0.4);
	  left_front.setPower(0.4);
	  right_back.setPower(0.4);
	  right_front.setPower(-0.4);
	  sleep(2550);
	  left_back.setPower(0);
	  left_front.setPower(0);
	  right_back.setPower(0);
	  right_front.setPower(0);
	  sleep(400);
	  left_back.setPower(0.4);
	  left_front.setPower(0.4);
	  right_back.setPower(0.4);
	  right_front.setPower(0.4);
	  sleep(3400);
	  left_back.setPower(0);
	  left_front.setPower(0);
	  right_back.setPower(0);
	  right_front.setPower(0);
	  sleep(400);
	  left_back.setPower(0.4);
	  left_front.setPower(-0.4);
	  right_back.setPower(-0.4);
	  right_front.setPower(0.4);
	  sleep(1300);
	  left_back.setPower(0);
	  left_front.setPower(0);
	  right_back.setPower(0);
	  right_front.setPower(0);
	  REDBACK();
	}
  }
  private void safeMoveToPosition(int targetPosition) {
		if (arm_1.isBusy()) {
			while (arm_1.isBusy() && opModeIsActive()) {
				telemetry.addData("Waiting for motor to reach position", arm_1.getTargetPosition());
				telemetry.addData("Current Position", arm_1.getCurrentPosition());
				telemetry.update();
			}
		}
		arm_1.setTargetPosition(targetPosition);
		arm_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		arm_1.setPower(0.6);

		while (arm_1.isBusy() && opModeIsActive()) {
			telemetry.addData("Target Position", targetPosition);
			telemetry.addData("Current Position", arm_1.getCurrentPosition());
			telemetry.update();
		}

		arm_1.setPower(0);

		arm_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}
  private void REDBACK() {
  safeMoveToPosition(948);
  sleep(1000);
  left_pixl_hand.setPosition(0);
  right_pixl_hand.setPosition(0.58);
  sleep(1000);
  left_pixl_hand.setPosition(0.3);
  right_pixl_hand.setPosition(0.8);
  sleep(1000);
  safeMoveToPosition(400);
  sleep(700);
  safeMoveToPosition(45);
  sleep(1000);
   left_back.setPower(-0.4);
	 left_front.setPower(-0.4);
	 right_back.setPower(-0.4);
	 right_front.setPower(-0.4);
  sleep(150);
  left_back.setPower(0);
	left_front.setPower(0);
	right_back.setPower(0);
	right_front.setPower(0);
	sleep(500);
  left_back.setPower(-0.4);
	left_front.setPower(0.4);
	right_back.setPower(0.4);
	right_front.setPower(-0.4);
	sleep(1200);
	left_back.setPower(0);
	left_front.setPower(0);
	right_back.setPower(0);
	right_front.setPower(0);
	sleep(200);
	left_back.setPower(0.4);
	  left_front.setPower(0.4);
	  right_back.setPower(0.4);
	  right_front.setPower(0.4);
	  sleep(350);
	  left_back.setPower(0);
	  left_front.setPower(0);
	  right_back.setPower(0);
	  right_front.setPower(0);
	requestOpModeStop();
  }
}
