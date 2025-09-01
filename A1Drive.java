package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class A1Drive {

	// ドライブ用モーター
	private DcMotorEx leftMotor;
	private DcMotorEx rightMotor;

	public A1Drive(HardwareMap hardwareMap) {
		// モーター初期化
		leftMotor = hardwareMap.get(DcMotorEx.class, "left_motor");
		rightMotor = hardwareMap.get(DcMotorEx.class, "right_motor");

		// モーター設定
		if (leftMotor != null) {
			leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			leftMotor.setDirection(DcMotor.Direction.FORWARD);
			leftMotor.setPower(0.0); // 確実に停止
		}
		if (rightMotor != null) {
			rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			rightMotor.setDirection(DcMotor.Direction.REVERSE); // 右モーターは逆方向に設定
			rightMotor.setPower(0.0); // 確実に停止
		}
	}

	// ドライブ制御メソッド（右スティック前後移動 + 右スティックX軸左右回転 + L1/R1左右回転）
	public void drive(double rightStickY, double rightStickX, boolean turnLeft, boolean turnRight) {
		// 厳密なデッドゾーン設定（より小さな値に設定）
		if (Math.abs(rightStickY) < 0.05) rightStickY = 0.0;
		if (Math.abs(rightStickX) < 0.05) rightStickX = 0.0;

		double leftPower = 0.0;
		double rightPower = 0.0;

		// 前後移動制御（右スティックY軸）
		if (Math.abs(rightStickY) >= 0.05) {
			leftPower = rightStickY;   // 左モーター（前進で正転）
			rightPower = rightStickY;  // 右モーター（前進で正転）
		}

		// 左右回転制御（右スティックX軸）
		if (Math.abs(rightStickX) >= 0.05) {
			// 右スティックX軸で左右回転
			leftPower += rightStickX;   // 左モーター（右回転で正転）
			rightPower -= rightStickX;  // 右モーター（右回転で逆転）
		}

		// 左右回転制御（L1/R1ボタン）
		if (turnLeft) {
			// 左回転（L1）- 左モーター正転、右モーター逆転
			leftPower += 0.5;
			rightPower -= 0.5;
		}
		if (turnRight) {
			// 右回転（R1）- 左モーター逆転、右モーター正転
			leftPower -= 0.5;
			rightPower += 0.5;
		}

		// パワー値を-1.0から1.0の範囲に制限
		leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
		rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

		// 安全機能：入力がない場合は確実に0.0に設定
		if (Math.abs(rightStickY) < 0.05 && Math.abs(rightStickX) < 0.05 && !turnLeft && !turnRight) {
			leftPower = 0.0;
			rightPower = 0.0;
		}

		// モーターにパワーを設定
		if (leftMotor != null) {
			leftMotor.setPower(leftPower);
		}
		if (rightMotor != null) {
			rightMotor.setPower(rightPower);
		}
	}

	// モーターの状態を取得
	public double getLeftMotorPower() {
		return leftMotor != null ? leftMotor.getPower() : 0.0;
	}

	public double getRightMotorPower() {
		return rightMotor != null ? rightMotor.getPower() : 0.0;
	}

	// モーターの接続状態を取得
	public boolean isLeftMotorConnected() {
		return leftMotor != null;
	}

	public boolean isRightMotorConnected() {
		return rightMotor != null;
	}
	
	// 緊急停止機能
	public void emergencyStop() {
		if (leftMotor != null) {
			leftMotor.setPower(0.0);
		}
		if (rightMotor != null) {
			rightMotor.setPower(0.0);
		}
	}
	
	// モーターの状態をリセット
	public void resetMotors() {
		if (leftMotor != null) {
			leftMotor.setPower(0.0);
			leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}
		if (rightMotor != null) {
			rightMotor.setPower(0.0);
			rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}
	}
} 