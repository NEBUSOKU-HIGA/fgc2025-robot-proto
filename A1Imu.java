package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class A1Imu {
	// IMU センサー
	private IMU imu;
	
	// SHARE/BACKでヘディング記憶 memorize
	private boolean prevSharePressed = false;
	private double storedHeadingDeg = 0.0;
	private boolean hasStoredHeading = false;
	
	// OPTIONS整向用のラッチ状態（目標到達後は押し続けても補正しない）
	private boolean prevOptionsPressed = false;
	private boolean headingLocked = false;
	private boolean isAutoTurnActive = false;
	private long autoTurnStartMs = 0L;
	private long autoTurnTimeoutMs = AUTO_TURN_TIMEOUT_MS; // 実行毎に設定される動的タイムアウト
	private int autoTurnPhase = 0; // 0:なし, 1:第1段, 2:第2段
	private boolean autoTurnPhaseHold = false; // フェーズ切替直後に一度停止
	
	// 自動整向の定数
	private static final long AUTO_TURN_TIMEOUT_MS = 2000; // options自動整向の最大制御時間
	private static final long AUTO_TURN_TIMEOUT_MS_MIN = 200;  // 目標付近の最短タイムアウト（近い時はより短く）
	private static final long AUTO_TURN_TIMEOUT_MS_MAX = 3000; // 反対側(≈180°)の最長タイムアウト（遠い時はより長く）
	private static final double AUTO_TURN_TIMEOUT_SCALE = 0.70; // タイムアウト全体スケール（70%）
	
	// 自動整向の回転出力スケーリング（角度差でKPと最大出力を補間）
	private static final double HEADING_KP_NEAR = 0.004; // 小角度用（ゆっくり）
	private static final double HEADING_KP_FAR  = 0.012; // 大角度用（強め）
	private static final double HEADING_MAX_NEAR = 0.20; // 小角度時の上限
	private static final double HEADING_MAX_FAR  = 0.80; // 大角度時の上限
	
	// エラー情報
	private String lastError = "";
	private boolean hasError = false;
	
	public A1Imu(HardwareMap hardwareMap) {
		initializeImu(hardwareMap);
	}
	
	private void initializeImu(HardwareMap hardwareMap) {
		try {
			imu = hardwareMap.get(IMU.class, "imu");
			if (imu != null) {
				IMU.Parameters params = new IMU.Parameters(
					new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.UP,
						RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
					)
				);
				imu.initialize(params);
				hasError = false;
				lastError = "";
				System.out.println("A1Imu: IMU initialized successfully");
			} else {
				hasError = true;
				lastError = "IMU not found in hardware map";
				System.out.println("A1Imu: ERROR - IMU not found in hardware map");
			}
		} catch (Exception e) {
			hasError = true;
			lastError = "IMU initialization failed: " + e.getMessage();
			System.out.println("A1Imu: ERROR - IMU initialization failed: " + e.getMessage());
		}
	}
	
	// ヘディング方向を文字列で取得
	public String getHeadingDirection(double yawDegrees) {
		double normalizedYaw = ((yawDegrees % 360) + 360) % 360;
		if (normalizedYaw >= 337.5 || normalizedYaw < 22.5) return "NORTH";
		if (normalizedYaw < 67.5) return "NORTHEAST";
		if (normalizedYaw < 112.5) return "EAST";
		if (normalizedYaw < 157.5) return "SOUTHEAST";
		if (normalizedYaw < 202.5) return "SOUTH";
		if (normalizedYaw < 247.5) return "SOUTHWEST";
		if (normalizedYaw < 292.5) return "WEST";
		return "NORTHWEST";
	}
	
	// 現在のYaw角度を取得
	public double getCurrentYaw() {
		if (imu != null) {
			try {
				return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
			} catch (Exception e) {
				hasError = true;
				lastError = "Error getting Yaw: " + e.getMessage();
				return 0.0;
			}
		}
		return 0.0;
	}
	
	// 現在のPitch角度を取得
	public double getCurrentPitch() {
		if (imu != null) {
			try {
				return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
			} catch (Exception e) {
				hasError = true;
				lastError = "Error getting Pitch: " + e.getMessage();
				return 0.0;
			}
		}
		return 0.0;
	}
	
	// 現在のRoll角度を取得
	public double getCurrentRoll() {
		if (imu != null) {
			try {
				return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
			} catch (Exception e) {
				hasError = true;
				lastError = "Error getting Roll: " + e.getMessage();
				return 0.0;
			}
		}
		return 0.0;
	}
	
	// 角速度Zを取得
	public double getAngularVelocityZ() {
		if (imu != null) {
			try {
				return imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
			} catch (Exception e) {
				hasError = true;
				lastError = "Error getting Angular Velocity: " + e.getMessage();
				return 0.0;
			}
		}
		return 0.0;
	}
	
	// SHAREボタンでヘディングを記憶
	public void updateHeadingMemory(boolean sharePressed) {
		if (imu != null) {
			double yawNowDeg = getCurrentYaw();
			if (sharePressed && !prevSharePressed) {
				storedHeadingDeg = yawNowDeg;
				hasStoredHeading = true;
				System.out.println("A1Imu: Heading saved: " + storedHeadingDeg + "°");
			}
		}
		prevSharePressed = sharePressed;
	}
	
	// OPTIONSボタンで自動整向を開始
	public void startAutoTurn(boolean optionsPressed) {
		if (optionsPressed && !prevOptionsPressed && hasStoredHeading && imu != null) {
			isAutoTurnActive = true;
			headingLocked = false;
			autoTurnStartMs = System.currentTimeMillis();
			
			// 初期誤差に応じてタイムアウト時間を設定（近いほど短く、遠いほど長く）
			double yawDegAtStart = getCurrentYaw();
			double errStart = (storedHeadingDeg - yawDegAtStart);
			errStart = ((errStart + 540) % 360) - 180;
			double absErrStart = Math.abs(errStart); // 0〜180
			double tRatio = Math.min(1.0, Math.max(0.0, absErrStart / 180.0));
			// 正マッピング: 近い→短い, 遠い→長い
			autoTurnTimeoutMs = AUTO_TURN_TIMEOUT_MS_MIN + Math.round((AUTO_TURN_TIMEOUT_MS_MAX - AUTO_TURN_TIMEOUT_MS_MIN) * tRatio);
			autoTurnTimeoutMs = Math.round(autoTurnTimeoutMs * AUTO_TURN_TIMEOUT_SCALE);
			autoTurnPhase = 1; // 第1段開始
			autoTurnPhaseHold = false;
			
			System.out.println("A1Imu: Auto turn started - Target: " + storedHeadingDeg + "°, Current: " + yawDegAtStart + "°, Error: " + errStart + "°");
		}
		prevOptionsPressed = optionsPressed;
	}
	
	// 自動整向の制御値を計算
	public double[] calculateAutoTurnControl() {
		double[] result = {0.0, 0.0}; // [leftPower, rightPower]
		
		if (!isAutoTurnActive || !hasStoredHeading || imu == null) {
			return result;
		}
		
		// タイムアウト判定
		long elapsed = System.currentTimeMillis() - autoTurnStartMs;
		if (elapsed >= autoTurnTimeoutMs) {
			result[0] = 0.0;
			result[1] = 0.0;
			if (autoTurnPhase == 1) {
				// 第1段タイムアウト直後：一旦停止して第2段を自動開始
				autoTurnPhase = 2;
				autoTurnPhaseHold = true; // 1サイクル停止
				// 第2段の開始時間とタイムアウト再設定（同じロジック）
				autoTurnStartMs = System.currentTimeMillis();
				double yawDegAtPhase2 = getCurrentYaw();
				double err2 = ((storedHeadingDeg + 180.0) - yawDegAtPhase2);
				err2 = ((err2 + 540) % 360) - 180;
				double absErr2 = Math.abs(err2);
				double r2 = Math.min(1.0, Math.max(0.0, absErr2 / 180.0));
				double inv2 = 1.0 - r2;
				autoTurnTimeoutMs = AUTO_TURN_TIMEOUT_MS_MIN + Math.round((AUTO_TURN_TIMEOUT_MS_MAX - AUTO_TURN_TIMEOUT_MS_MIN) * inv2);
				autoTurnTimeoutMs = Math.round(autoTurnTimeoutMs * AUTO_TURN_TIMEOUT_SCALE);
				
				System.out.println("A1Imu: Auto turn phase 2 started - Target: " + (storedHeadingDeg + 180.0) + "°, Current: " + yawDegAtPhase2 + "°, Error: " + err2 + "°");
			} else {
				// 第2段も終了 → 完全停止
				isAutoTurnActive = false;
				headingLocked = true;
				System.out.println("A1Imu: Auto turn completed");
			}
		} else {
			// すでにロック済みなら左右を常に停止
			if (headingLocked) {
				result[0] = 0.0;
				result[1] = 0.0;
			} else {
				double yawDeg = getCurrentYaw();
				double error = ((storedHeadingDeg + 180.0) - yawDeg);
				// -180..+180 に正規化
				error = ((error + 540) % 360) - 180;
				double absErr = Math.abs(error);
				
				// フェーズ切替直後は一度停止してから反転方向へ
				if (autoTurnPhaseHold) {
					result[0] = 0.0;
					result[1] = 0.0;
					autoTurnPhaseHold = false; // 次サイクルから制御
				} else {
					if (absErr <= 2.0) { // 許容誤差±2°
						result[0] = 0.0;
						result[1] = 0.0;
						headingLocked = true; // 以降は押し続けても補正しない
						isAutoTurnActive = false; // 自動整向完了
						System.out.println("A1Imu: Auto turn target reached - Error: " + absErr + "°");
					} else {
						// 誤差に比例（角度差に応じてKP/上限を補間）
						double ratio = Math.min(1.0, Math.max(0.0, absErr / 180.0));
						double kp = HEADING_KP_NEAR + (HEADING_KP_FAR - HEADING_KP_NEAR) * ratio;
						double maxOut = HEADING_MAX_NEAR + (HEADING_MAX_FAR - HEADING_MAX_NEAR) * ratio;
						double cmd = kp * absErr;
						if (cmd > maxOut) cmd = maxOut;   // 上限
						double rot = Math.copySign(cmd, error); // error>0 なら左回り
						// ユーザー指定：left=-rot, right=-rot
						result[0] = -rot;
						result[1] = -rot;
					}
				}
			}
		}
		
		return result;
	}
	
	// 手動入力で自動整向をキャンセル
	public void cancelAutoTurn(boolean hasManualInput) {
		if (isAutoTurnActive && hasManualInput) {
			isAutoTurnActive = false;
			headingLocked = false;
			autoTurnPhase = 0;
			autoTurnPhaseHold = false;
			System.out.println("A1Imu: Auto turn cancelled by manual input");
		}
	}
	
	// 自動整向がアクティブかどうか
	public boolean isAutoTurnActive() {
		return isAutoTurnActive;
	}
	
	// ヘディングがロックされているかどうか
	public boolean isHeadingLocked() {
		return headingLocked;
	}
	
	// 記憶したヘディングがあるかどうか
	public boolean hasStoredHeading() {
		return hasStoredHeading;
	}
	
	// 記憶したヘディングを取得
	public double getStoredHeading() {
		return storedHeadingDeg;
	}
	
	// IMUが初期化されているかどうか
	public boolean isInitialized() {
		return imu != null && !hasError;
	}
	
	// エラーがあるかどうか
	public boolean hasError() {
		return hasError;
	}
	
	// 最後のエラーメッセージを取得
	public String getLastError() {
		return lastError;
	}
	
	// IMUのYawをリセット
	public void resetYaw() {
		if (imu != null) {
			try {
				imu.resetYaw();
				hasError = false;
				lastError = "";
				System.out.println("A1Imu: Yaw reset successfully");
			} catch (Exception e) {
				hasError = true;
				lastError = "Yaw reset failed: " + e.getMessage();
				System.out.println("A1Imu: ERROR - Yaw reset failed: " + e.getMessage());
			}
		}
	}
} 
