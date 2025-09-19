package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class A1Arm {
	
	// アーム用モーター
	private DcMotorEx armMotor;
	
	public A1Arm(HardwareMap hardwareMap) {
		// アームモーター初期化
		armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
		
		// デバッグ情報
		System.out.println("Arm Motor Initialization - armMotor: " + (armMotor != null ? "Connected" : "NULL"));
		
		// モーター設定
		if (armMotor != null) {
			armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			armMotor.setDirection(DcMotor.Direction.FORWARD);
			armMotor.setPower(0.0); // 初期状態で停止
			System.out.println("Arm Motor Setup Complete");
		}
	}
	
	// アーム制御メソッド（L2ボタン）
	public void moveArm(double leftTrigger) {
		// デッドゾーン設定（より敏感に）
		if (leftTrigger < 0.05) leftTrigger = 0.0;
		
		// アームモーター制御
		if (armMotor != null) {
			armMotor.setPower(leftTrigger);
			// デバッグ情報
			System.out.println("Arm Motor - Trigger: " + leftTrigger + ", Power: " + leftTrigger);
		} else {
			System.out.println("Arm Motor - NULL");
		}
	}
	
	// アームモーターの状態を取得
	public double getArmMotorPower() {
		return armMotor != null ? armMotor.getPower() : 0.0;
	}
	
	// アームモーターの接続状態を取得
	public boolean isArmMotorConnected() {
		return armMotor != null;
	}
	
	// アームモーター 取得
	public DcMotorEx getArmMotor() {
		return armMotor;
	}

	// テスト用：強制動作
	public void testArmMotor() {
		if (armMotor != null) {
			System.out.println("Testing Arm Motor - Setting power to 0.5");
			armMotor.setPower(0.5);
			System.out.println("Arm Motor Power after test: " + armMotor.getPower());
		}
	}
	
	// テスト用：停止
	public void stopArmMotor() {
		if (armMotor != null) {
			System.out.println("Stopping Arm Motor");
			armMotor.setPower(0.0);
		}
	}
	
	// 固定パワーでアームモーターを動かす
	public void moveArmWithFixedPower(double power) {
		if (armMotor != null) {
			// パワー値を-1.0から1.0の範囲に制限
			power = Math.max(-1.0, Math.min(1.0, power));
			
			armMotor.setPower(power);
			System.out.println("Arm Motor Fixed Power: " + power);
		} else {
			System.out.println("Arm Motor - NULL (Fixed Power)");
		}
	}
} 
