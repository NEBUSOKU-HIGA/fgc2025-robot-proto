package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class A1Servo {
	
	// サーボモーター
	private Servo servo1;
	private Servo servo2;
	
	// サーボの位置制限
	private static final double MIN_POSITION = 0.250; // M1 servo1 위치
	private static final double MAX_POSITION = 0.850; // M1 servo2 위치
	private static final double SERVO1_MAX_POSITION = 0.700; // M2 servo1 위치
	private static final double POSITION_INCREMENT = 0.05;
	
	public A1Servo(HardwareMap hardwareMap) {
		// サーボモーター初期化
		servo1 = hardwareMap.get(Servo.class, "servo1");
		servo2 = hardwareMap.get(Servo.class, "servo2");
		
		// サーボの初期設定
		if (servo1 != null) {
			servo1.setDirection(Servo.Direction.FORWARD);
			servo1.setPosition(0.475); // 中央位置に初期化（0.250-0.700の範囲内）
		}
		if (servo2 != null) {
			servo2.setDirection(Servo.Direction.FORWARD);
			servo2.setPosition(0.700); // 中央位置に初期化（0.500-0.900の範囲内）
		}
	}
	
	// servo1の位置を設定
	public void setServo1Position(double position) {
		if (servo1 != null) {
			double currentPosition = getServo1Position();
			System.out.println("A1Servo - setServo1Position called with: " + position);
			System.out.println("A1Servo - Current servo1 position: " + currentPosition);
			double originalPosition = position;
			position = Math.max(MIN_POSITION, Math.min(SERVO1_MAX_POSITION, position));
			if (originalPosition != position) {
				System.out.println("A1Servo - Position limited from " + originalPosition + " to " + position);
			}
			servo1.setPosition(position);
			System.out.println("A1Servo - servo1.setPosition(" + position + ") executed");
			
			
			if (currentPosition >= 0.65 && position <= 0.3) {
				System.out.println("*** SERVO1 POSITION CHANGE DETECTED: " + currentPosition + " -> " + position + " ***");
				System.out.println("*** This change should be investigated ***");
			}
		}
	}
	
	// servo2の位置を設定
	public void setServo2Position(double position) {
		if (servo2 != null) {
			position = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));
			servo2.setPosition(position);
		}
	}
	
	// servo1の位置を取得
	public double getServo1Position() {
		return servo1 != null ? servo1.getPosition() : 0.0;
	}
	
	// servo2の位置を取得
	public double getServo2Position() {
		return servo2 != null ? servo2.getPosition() : 0.0;
	}
	
	// servo1の接続状態を取得
	public boolean isServo1Connected() {
		return servo1 != null;
	}
	
	// servo2の接続状態を取得
	public boolean isServo2Connected() {
		return servo2 != null;
	}
	
	// servo1 object
	public Servo getServo1() {
		return servo1;
	}
	
	// servo2 object
	public Servo getServo2() {
		return servo2;
	}
	
	// サーボの位置を増加
	public void increaseServo1Position() {
		if (servo1 != null) {
			double newPosition = Math.min(SERVO1_MAX_POSITION, getServo1Position() + POSITION_INCREMENT);
			setServo1Position(newPosition);
		}
	}
	
	// サーボの位置を減少
	public void decreaseServo1Position() {
		if (servo1 != null) {
			double newPosition = Math.max(MIN_POSITION, getServo1Position() - POSITION_INCREMENT);
			setServo1Position(newPosition);
		}
	}
	
	// servo2の位置を増加
	public void increaseServo2Position() {
		if (servo2 != null) {
			double newPosition = Math.min(MAX_POSITION, getServo2Position() + POSITION_INCREMENT);
			setServo2Position(newPosition);
		}
	}
	
	// servo2の位置を減少
	public void decreaseServo2Position() {
		if (servo2 != null) {
			double newPosition = Math.max(MIN_POSITION, getServo2Position() - POSITION_INCREMENT);
			setServo2Position(newPosition);
		}
	}

	// 十字キーによるサーボ制御
	public void controlServosWithDpad(boolean dpadLeft, boolean dpadRight, boolean dpadUp, boolean dpadDown) {
		// servo1の制御（左右）
		if (dpadLeft) {
			System.out.println("A1Servo - D-pad Left pressed, decreasing servo1");
			decreaseServo1Position();
		}
		if (dpadRight) {
			System.out.println("A1Servo - D-pad Right pressed, increasing servo1");
			increaseServo1Position();
		}
		
		// servo2の制御（上下）
		if (dpadUp) {
			System.out.println("A1Servo - D-pad Up pressed, increasing servo2");
			increaseServo2Position();
		}
		if (dpadDown) {
			System.out.println("A1Servo - D-pad Down pressed, decreasing servo2");
			decreaseServo2Position();
		}
	}
	
	// M1ボタンで両servo control (servo1: 0.25, servo2: 0.85)
	public void moveServosToM1Position() {
		System.out.println("A1Servo - moveServosToM1Position called");
		setServo1Position(0.25); // servo1 M1
	}

	// M2ボタンで両servo control (servo1: 0.7, servo2: 0.5)
	public void moveServosToM2Position() {
		System.out.println("A1Servo - moveServosToM2Position called");
		setServo2Position(0.5); // servo2 M2
	}

	// M2 button servo1 setup (independent from other controls)
	public void forceServo1ToM2Position() {
		System.out.println("A1Servo - forceServo1ToM2Position called");
		if (servo1 != null) {
			double currentPos = getServo1Position();
			System.out.println("A1Servo - Current servo1 position before force: " + currentPos);
			System.out.println("A1Servo - servo1.setPosition(0.7) executed directly");
			System.out.println("A1Servo - New servo1 position after force: " + getServo1Position());
		}
	}

	// Aボタンで両 servoを最小位置に移動
	public void moveBothServosToMin() {
		System.out.println("A1Servo - moveBothServosToMin called (A button)");
		setServo1Position(0.25); // servo1 min
		setServo2Position(0.5); // servo2 max
	}

	// Bボタンで両 servoを最大位置に移動
	public void moveBothServosToMax() {
		System.out.println("A1Servo - moveBothServosToMax called (B button)");

	}
	
	// M1ボタンでservo1を特定位置に移動
	public void moveServo1ToM1Position() {
		setServo1Position(0.25); // M1 : 0.25
	}
	
	// M2ボタンでservo2를 特定位置に移動
	public void moveServo2ToM2Position() {
		setServo2Position(0.5); // M2 : 0.5
	}
	
	// M1, M2ボタンで両 servo move to location
	public void moveServosToM1M2Position() {
		moveServo1ToM1Position(); // servo1 M1
		moveServo2ToM2Position(); // servo2 M2
	}
} 
