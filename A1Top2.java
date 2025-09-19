package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class A1Top2 {
	
	// トップ用モーター
	private DcMotorEx topMotor;
	
	// トップモーター用の変数
	private double currentTopPower = 0.0;
	private double targetTopPower = 0.0;
	private static final double POWER_INCREMENT = 0.05; // 5%ずつ増加
	private static final double POWER_DECREMENT = 0.1; // 10%ずつ減少
	
	public A1Top2(HardwareMap hardwareMap) {
		// トップモーター初期化
		topMotor = hardwareMap.get(DcMotorEx.class, "top_motor");
		
		// モーター設定
		if (topMotor != null) {
			topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			topMotor.setDirection(DcMotor.Direction.REVERSE);
			topMotor.setPower(0.0); // 初期状態で停止
		}
	}
	
	// トップモーター制御メソッド（R2トリガー）
	public void moveTop(double rightTrigger) {
		// デッドゾーン設定
		if (rightTrigger < 0.05) rightTrigger = 0.0;
		
		// 目標パワーを設定
		targetTopPower = rightTrigger;
		
		// 徐々にパワーを調整
		if (targetTopPower > currentTopPower) {
			// パワーを増加（ゆっくり）
			currentTopPower = Math.min(targetTopPower, currentTopPower + POWER_INCREMENT);
		} else if (targetTopPower < currentTopPower) {
			// パワーを減少（少し速く）
			currentTopPower = Math.max(targetTopPower, currentTopPower - POWER_DECREMENT);
		}
		
		// トップモーター制御
		if (topMotor != null) {
			topMotor.setPower(currentTopPower);
		}
	}
	
	// R2トリガーでM1コンボを判定するメソッド
	public void moveTopWithM1Combo(double rightTrigger) {
		if (rightTrigger > 0.1) {
			// M1コンボ実行
			moveTopM1Combo();
		} else {
			// 通常の制御
			moveTop(rightTrigger);
		}
	}
	
	// M1コンボ用トップモーター制御（R2トリガー強押し時）
	public void moveTopM1Combo() {
		if (topMotor != null) {
			// コンボ時は強制的に一定速度で回す
			currentTopPower = 1.0;
			targetTopPower = 1.0;
			topMotor.setPower(currentTopPower);
		}
	}
	
	public void moveTopM1ComboReverse() {
		if (topMotor != null) {
			// コンボ時は強制的に一定速度で反対方向に回す
			currentTopPower = -1.0;
			targetTopPower = -1.0;
			topMotor.setPower(currentTopPower);
		}
	}
	
	// R2トリガーでM1コンボを判定するメソッド（反対方向）
	public void moveTopWithM1ComboReverse(double rightTrigger) {
		if (rightTrigger > 0.1) {
			// M1コンボ実行（反対方向）
			moveTopM1ComboReverse();
		} else {
			// 通常の制御（反対方向）
			moveTop(rightTrigger, true); // reverseDirection = true
		}
	}
	
	// トップモーター制御メソッド（R2ボタン + 反対方向フラグ）
	public void moveTop(double rightTrigger, boolean reverseDirection) {
		// デッドゾーン設定
		if (rightTrigger < 0.05) rightTrigger = 0.0;
		
		// 反対方向の場合は負の値にする
		if (reverseDirection) {
			rightTrigger = -rightTrigger;
		}
		
		// 目標パワーを設定
		targetTopPower = rightTrigger;
		
		// 徐々にパワーを調整
		if (targetTopPower > currentTopPower) {
			// パワーを増加（ゆっくり）
			currentTopPower = Math.min(targetTopPower, currentTopPower + POWER_INCREMENT);
		} else if (targetTopPower < currentTopPower) {
			// パワーを減少（少し速く）
			currentTopPower = Math.max(targetTopPower, currentTopPower - POWER_DECREMENT);
		}
		
		// トップモーター制御
		if (topMotor != null) {
			topMotor.setPower(currentTopPower);
		}
	}
	
	// トップモーターの状態を取得
	public double getTopMotorPower() {
		return topMotor != null ? topMotor.getPower() : 0.0;
	}
	
	// トップモーターの接続状態を取得
	public boolean isTopMotorConnected() {
		return topMotor != null;
	}
	
	// 現在のパワー値を取得
	public double getCurrentTopPower() {
		return currentTopPower;
	}
	
	// 目標パワー値を取得
	public double getTargetTopPower() {
		return targetTopPower;
	}
	
	// パワー増加値を取得
	public double getPowerIncrement() {
		return POWER_INCREMENT;
	}
	
	// パワー減少値を取得
	public double getPowerDecrement() {
		return POWER_DECREMENT;
	}
	
	// テスト用：強制動作
	public void testTopMotor() {
		if (topMotor != null) {
			System.out.println("Testing Top Motor - Setting power to 0.5");
			topMotor.setPower(0.5);
			System.out.println("Top Motor Power after test: " + topMotor.getPower());
		}
	}
	
	// テスト用：停止
	public void stopTopMotor() {
		if (topMotor != null) {
			System.out.println("Stopping Top Motor");
			topMotor.setPower(0.0);
		}
	}
} 
