/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="motorTest", group="TeleOp")
public class motorTest extends LinearOpMode {

	// モーターの宣言
	private DcMotor leftMotor;
	private DcMotor rightMotor;
	private DcMotor topMotor;

	@Override
	public void runOpMode() {
		// モーターの初期化
		leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
		rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
		topMotor = hardwareMap.get(DcMotor.class, "top_motor");

		// モーターの方向設定
		leftMotor.setDirection(DcMotor.Direction.FORWARD);
		rightMotor.setDirection(DcMotor.Direction.FORWARD);
		topMotor.setDirection(DcMotor.Direction.FORWARD);

		// 初期化完了を待つ
		waitForStart();

		while (opModeIsActive()) {
			// 右Joystickの値を取得
			float rightY = gamepad1.right_stick_y;
			float rightX = gamepad1.right_stick_x;
			
			// 左Joystickの値を取得
			float leftY = gamepad1.left_stick_y;

			// デッドゾーン設定（0.1以下は無視）
			if (Math.abs(rightY) < 0.1) {
				rightY = 0.0f;
			}
			if (Math.abs(rightX) < 0.1) {
				rightX = 0.0f;
			}
			if (Math.abs(leftY) < 0.1) {
				leftY = 0.0f;
			}

			// モーター制御
			// Y軸前進: left_motor正回転
			leftMotor.setPower(rightY);
			
			// X軸右: right_motor正回転
			rightMotor.setPower(rightX);
			
			// 左Joystick前後: top_motor制御
			topMotor.setPower(leftY);

			// テレメトリ表示
			telemetry.addData("Left Motor Power", "%.2f", leftMotor.getPower());
			telemetry.addData("Right Motor Power", "%.2f", rightMotor.getPower());
			telemetry.addData("Top Motor Power", "%.2f", topMotor.getPower());
			telemetry.addData("Right Stick Y", "%.2f", rightY);
			telemetry.addData("Right Stick X", "%.2f", rightX);
			telemetry.addData("Left Stick Y", "%.2f", leftY);
			telemetry.update();

			sleep(10); // 100Hz loop
		}
	}
	
} 
*/