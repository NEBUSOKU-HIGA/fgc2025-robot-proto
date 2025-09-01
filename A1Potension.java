package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class A1Potension {
	
	// ポテンショメーター制御用変数
	private static final double MIN_HEIGHT = 0.326;
	private static final double MAX_HEIGHT = 0.7;
	private static final double TARGET_HEIGHT = 0.78;
	private static final double HEIGHT_TOLERANCE = 0.02;
	private boolean autoHeightControl = false;
	
	// アナログ入力
	private AnalogInput potentiometer;
	
	public A1Potension(HardwareMap hardwareMap) {
		// アナログ入力初期化
		potentiometer = hardwareMap.get(AnalogInput.class, "poten");
	}
	
	// ポテンショメーターの値を読み取り
	public double getCurrentHeight() {
		if (potentiometer != null) {
			return potentiometer.getVoltage() / potentiometer.getMaxVoltage();
		}
		return 0.0;
	}
	
	// 段階的制限を適用したパワーを計算
	public double getAdjustedLiftPower(double originalPower) {
		double currentHeight = getCurrentHeight();
		double adjustedPower = originalPower;
		
		if (currentHeight >= TARGET_HEIGHT) {
			// 0.780以上で完全停止
			adjustedPower = 0.0;
		} else if (currentHeight > 0.3 && currentHeight < TARGET_HEIGHT) {
			// 0.3-0.78で段階的に制限（0.3で100%、0.78で0%）
			double reductionFactor = (TARGET_HEIGHT - currentHeight) / (TARGET_HEIGHT - 0.3);
			reductionFactor = Math.max(0.0, Math.min(1.0, reductionFactor)); // 0.0-1.0に制限
			adjustedPower = originalPower * reductionFactor;
		}
		
		return adjustedPower;
	}
	
	// 自動高さ制御のパワーを計算
	public double getAutoControlPower() {
		double currentHeight = getCurrentHeight();
		double heightError = TARGET_HEIGHT - currentHeight;
		
		if (Math.abs(heightError) > HEIGHT_TOLERANCE) {
			// 目標値から外れている場合、フィードバック制御
			double controlPower = Math.max(-0.5, Math.min(0.5, heightError * 2.0)); // 比例制御
			// 段階的制限を適用
			if (currentHeight >= TARGET_HEIGHT) {
				controlPower = 0.0;
			} else if (currentHeight > 0.3) {
				double reductionFactor = (TARGET_HEIGHT - currentHeight) / (TARGET_HEIGHT - 0.3);
				reductionFactor = Math.max(0.0, Math.min(1.0, reductionFactor));
				controlPower *= reductionFactor;
			}
			return controlPower;
		} else {
			// 目標値内の場合、停止
			return 0.0;
		}
	}
	
	// 自動制御の切り替え
	public void setAutoHeightControl(boolean enabled) {
		autoHeightControl = enabled;
	}
	
	// 自動制御の状態を取得
	public boolean isAutoHeightControlEnabled() {
		return autoHeightControl;
	}
	
	// ポテンショメーターの接続状態を取得
	public boolean isPotentiometerConnected() {
		return potentiometer != null;
	}
	
	// ポテンショメーターの電圧を取得
	public double getPotentiometerVoltage() {
		return potentiometer != null ? potentiometer.getVoltage() : 0.0;
	}
	
	// 減少係数を取得
	public double getReductionFactor() {
		double currentHeight = getCurrentHeight();
		if (currentHeight >= TARGET_HEIGHT) {
			return 0.0; // 0.780以上で完全停止
		} else if (currentHeight > 0.3 && currentHeight < TARGET_HEIGHT) {
			return (TARGET_HEIGHT - currentHeight) / (TARGET_HEIGHT - 0.3);
		}
		return 1.0;
	}
	
	// 高さ誤差を取得
	public double getHeightError() {
		return TARGET_HEIGHT - getCurrentHeight();
	}
	
	// 定数を取得
	public double getMaxHeight() { return MAX_HEIGHT; }
	public double getTargetHeight() { return TARGET_HEIGHT; }
	public double getHeightTolerance() { return HEIGHT_TOLERANCE; }
} 