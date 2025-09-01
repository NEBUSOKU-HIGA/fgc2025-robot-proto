/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.hardware.IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name="CompleteController", group="Test")
public class NewTaskScheduler extends LinearOpMode {

	private static final String SERVER_URL = "http://192.168.43.48:5000";
	private static final String JOYSTICK1_ENDPOINT = "/joystick1";
	private static final String JOYSTICK2_ENDPOINT = "/joystick2";
	private static final String DISTANCE_ENDPOINT = "/distance";
	private static final String COLOR_ENDPOINT = "/color";
	private static final String ROBOT_CONTROL_ENDPOINT = "/robot_control";
	// private static final String IMU_ENDPOINT = "/imu";
	// private static final String RECORD_ENDPOINT = "/record";
	// private static final String PLAYBACK_ENDPOINT = "/playback";
	
	// Distance Sensor 객체
	private DistanceSensor distanceSensor;
	
	// Color Sensor 객체 (REV Color Sensor V3)
	private ColorSensor colorSensor;
	
	// HD Hex Motor 객체
	private DcMotorEx coreHexMotor;
	
	// Wheel Motors 객체
	private DcMotor leftMotor;
	private DcMotor rightMotor;
	
	// Expansion Hub Motors 객체
	private DcMotorEx armMotor;
	private DcMotor liftMotor;
	
	// Distance Sensor 설정
	private static final double WARNING_DISTANCE_CM = 30.0; // 30cmで警告
	private static final double CRITICAL_DISTANCE_CM = 15.0; // 15cmで危険
	
	// HTTP 서버 관련 변수들
	private ServerSocket serverSocket;
	private AtomicBoolean serverRunning = new AtomicBoolean(false);
	private AtomicReference<Float> receivedMotorPower = new AtomicReference<>(0.0f);
	private int robotControlPort = 5001; // 로봇 제어용 포트
	
	// 파일 정보 관련 변수들
	private String lastFileName = "";
	private int lastFrameCount = 0;
	private double lastDuration = 0.0;
	private long lastFileSize = 0;
	private String lastUploadTime = "";
	private String fileInfoStatus = "No file uploaded";
	
	// IMU 객체 (コメントアウト)
	// private IMU imu;
	
	// 記録関連の変数 (Webダッシュボードで制御するため簡素化)
	// private boolean isRecording = false;
	// private List<JoystickFrame> recordedFrames = new ArrayList<>();
	// private long recordStartTime = 0;
	
	// Joystick 1 통계 변수들
	private int joystick1SendCount = 0;
	private int joystick1SuccessCount = 0;
	private int joystick1ErrorCount = 0;
	private String joystick1LastError = "";
	
	// Joystick 2 통계 변수들
	private int joystick2SendCount = 0;
	private int joystick2SuccessCount = 0;
	private int joystick2ErrorCount = 0;
	private String joystick2LastError = "";
	
	// Distance Sensor 통계 변수들
	private int distanceSendCount = 0;
	private int distanceSuccessCount = 0;
	private int distanceErrorCount = 0;
	private String distanceLastError = "";
	
	// Color Sensor 통계 변수들
	private int colorSendCount = 0;
	private int colorSuccessCount = 0;
	private int colorErrorCount = 0;
	private String colorLastError = "";
	
	// Robot Control 통계 변수들
	private int robotControlReceiveCount = 0;
	private int robotControlSuccessCount = 0;
	private int robotControlErrorCount = 0;
	private String robotControlLastError = "";
	
	// IMU 통계 변수들 (コメントアウト)
	// private int imuSendCount = 0;
	// private int imuSuccessCount = 0;
	// private int imuErrorCount = 0;
	// private String imuLastError = "";

	// ジョイスティックフレームクラス (コメントアウト - Webダッシュボードで処理)
	// private static class JoystickFrame {
	//	 long timestamp;
	//	 float leftX1, leftY1, rightX1, rightY1;
	//	 float leftX2, leftY2, rightX2, rightY2;
	//	 
	//	 JoystickFrame(long timestamp, float leftX1, float leftY1, float rightX1, float rightY1,
	//				  float leftX2, float leftY2, float rightX2, float rightY2) {
	//		 this.timestamp = timestamp;
	//		 this.leftX1 = leftX1;
	//		 this.leftY1 = leftY1;
	//		 this.rightX1 = rightX1;
	//		 this.rightY1 = rightY1;
	//		 this.leftX2 = leftX2;
	//		 this.leftY2 = leftY2;
	//		 this.rightX2 = rightX2;
	//		 this.rightY2 = rightY2;
	//	 }
	// }

	@Override
	public void runOpMode() {
		// Distance Sensor 초기화
		initializeDistanceSensor();
		
		// Color Sensor 초기화
		initializeColorSensor();
		
		// Wheel Motors 초기화
		initializeWheelMotors();
		
		// Core Hex Motor 초기화
		initializeCoreHexMotor();
		
		// Expansion Hub Motors 초기화
		initializeExpansionHubMotors();
		
		// HTTP 서버 시작
		startRobotControlServer();
		
		// IMU 초기화 (コメントアウト)
		// initializeIMU();
		
		telemetry.addData("Status", "Complete Controller Initialized");
		telemetry.addData("Target URL", SERVER_URL);
		telemetry.addData("Joystick 1 Endpoint", JOYSTICK1_ENDPOINT);
		telemetry.addData("Joystick 2 Endpoint", JOYSTICK2_ENDPOINT);
		telemetry.addData("Distance Endpoint", DISTANCE_ENDPOINT);
		telemetry.addData("Color Endpoint", COLOR_ENDPOINT);
		telemetry.addData("Robot Control Endpoint", ROBOT_CONTROL_ENDPOINT);
		telemetry.addData("Robot Control Server", "Port " + robotControlPort);
		// telemetry.addData("IMU Endpoint", IMU_ENDPOINT);
		// telemetry.addData("Record Endpoint", RECORD_ENDPOINT);
		// telemetry.addData("Playback Endpoint", PLAYBACK_ENDPOINT);
		// telemetry.addData("IMU", "BHI260AP");
		telemetry.addData("Distance Sensor", "REV 2M Distance Sensor");
		telemetry.addData("Color Sensor", "REV Color Sensor V3");
		telemetry.addData("HD Hex Motor", "R2 Trigger control");
		telemetry.addData("Wheel Motors", "Right Joystick Y-axis + L1/R1 rotation");
		telemetry.addData("Arm Motor", "L2 Trigger control");
		telemetry.addData("Lift Motor", "Left Joystick Y-axis control");
		telemetry.addData("Warning Distance", "%.1f cm", WARNING_DISTANCE_CM);
		telemetry.addData("Critical Distance", "%.1f cm", CRITICAL_DISTANCE_CM);
		telemetry.addData("Recording", "Use Web Dashboard to control recording");
		telemetry.addData("Web Dashboard", "http://192.168.43.48:5000");
		telemetry.addData("Note", "Robot sends joystick & distance data continuously");
		telemetry.addData("Robot Control", "Receives motor commands from web dashboard");
		telemetry.addData("File Upload", "Supports template file upload and playback");
		telemetry.update();

		waitForStart();

		while (opModeIsActive()) {
			// Get joystick values for Gamepad 1
			float leftX1 = gamepad1.left_stick_x;
			float leftY1 = -gamepad1.left_stick_y;
			float rightX1 = gamepad1.right_stick_x;
			float rightY1 = -gamepad1.right_stick_y;
			
			// Get joystick values for Gamepad 2
			float leftX2 = gamepad2.left_stick_x;
			float leftY2 = -gamepad2.left_stick_y;
			float rightX2 = gamepad2.right_stick_x;
			float rightY2 = -gamepad2.right_stick_y;
			
			// Get distance sensor value
			double distanceCm = getDistance();
			
			// HD Hex Motor 제어 (R2 Trigger)
			controlCoreHexMotor(gamepad1.right_trigger);
			
			// Wheel Motors 제어 (Right Joystick Y-axis for forward/backward + L1/R1 for rotation)
			controlWheelMotors(rightY1, gamepad1.left_bumper, gamepad1.right_bumper);
			
			// Expansion Hub Motors 제어 (L2 trigger + Left Joystick Y-axis)
			controlExpansionHubMotors(gamepad1.left_trigger, leftY1);
			
			// ロボット制御コマンドの確認
			checkRobotControlCommands();
			
			// 記録機能の制御 (Webダッシュボードで制御するため削除)
			// if (gamepad1.a && !isRecording) {
			//	 startRecording();
			// } else if (gamepad1.a && isRecording) {
			//	 stopRecording();
			// }
			
			// 再生機能の制御 (Webダッシュボードで制御するため削除)
			// if (gamepad1.b && !isRecording && !recordedFrames.isEmpty()) {
			//	 startPlayback();
			// }
			
			// 記録中の場合、フレームを記録 (Webダッシュボードで処理するため削除)
			// if (isRecording) {
			//	 recordFrame(leftX1, leftY1, rightX1, rightY1, leftX2, leftY2, rightX2, rightY2);
			// }
			
			// Send joystick 1 data every 100ms (10Hz)
			if (joystick1SendCount % 10 == 0) {
				sendJoystick1Data(leftX1, leftY1, rightX1, rightY1);
			}
			joystick1SendCount++;
			
			// Send joystick 2 data every 100ms (10Hz)
			if (joystick2SendCount % 10 == 0) {
				sendJoystick2Data(leftX2, leftY2, rightX2, rightY2);
			}
			joystick2SendCount++;
			
			// Send distance data every 100ms (10Hz)
			if (distanceSendCount % 10 == 0) {
				sendDistanceData(distanceCm);
			}
			distanceSendCount++;
			
			// Send color sensor data every 100ms (10Hz)
			if (colorSendCount % 10 == 0) {
				sendColorSensorData();
			}
			colorSendCount++;
			
			// Send IMU data every 100ms (10Hz) (コメントアウト)
			// if (imuSendCount % 10 == 0) {
			//	 sendIMUData();
			// }
			// imuSendCount++;
			
			// Display telemetry
			displayTelemetry(leftX1, leftY1, rightX1, rightY1, leftX2, leftY2, rightX2, rightY2, distanceCm);
			
			sleep(10); // 100Hz loop
		}
		
		// HTTP 서버 정리
		stopRobotControlServer();
	}

	// Distance Sensor 초기화
	private void initializeDistanceSensor() {
		try {
			telemetry.addData("DEBUG", "Attempting to get distanceSensor from hardwareMap...");
			distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
			telemetry.addData("Distance Sensor Status", "Initialized Successfully");
			telemetry.addData("DEBUG", "Distance Sensor object: " + (distanceSensor != null ? "NOT NULL" : "NULL"));
		} catch (Exception e) {
			telemetry.addData("Distance Sensor Status", "Initialization Failed: " + e.getMessage());
			telemetry.addData("DEBUG", "Distance Sensor Exception: " + e.toString());
			distanceLastError = "Distance Sensor Init Error: " + e.getMessage();
		}
	}
	
	// Color Sensor 초기화
	private void initializeColorSensor() {
		try {
			telemetry.addData("DEBUG", "Attempting to get colorSensor from hardwareMap...");
			colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
			// LEDを有効化
			if (colorSensor != null) {
				colorSensor.enableLed(true);
				telemetry.addData("DEBUG", "Color Sensor LED enabled");
			}
			telemetry.addData("Color Sensor Status", "Initialized Successfully");
			telemetry.addData("DEBUG", "Color Sensor object: " + (colorSensor != null ? "NOT NULL" : "NULL"));
		} catch (Exception e) {
			telemetry.addData("Color Sensor Status", "Initialization Failed: " + e.getMessage());
			telemetry.addData("DEBUG", "Color Sensor Exception: " + e.toString());
			colorLastError = "Color Sensor Init Error: " + e.getMessage();
		}
	}
	
	// HD Hex Motor 초기화
	private void initializeCoreHexMotor() {
		try {
			coreHexMotor = hardwareMap.get(DcMotorEx.class, "top_motor");
			coreHexMotor.setDirection(DcMotor.Direction.REVERSE);
			coreHexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			coreHexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			telemetry.addData("HD Hex Motor Status", "Initialized as DcMotorEx");
		} catch (Exception e) {
			telemetry.addData("HD Hex Motor Init Error", e.getMessage());
		}
	}
	
	// Wheel Motors 초기화
	private void initializeWheelMotors() {
		try {
			leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
			rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
			
			// モーターのゼロパワー動作を設定
			leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			
			telemetry.addData("Wheel Motors Status", "Initialized Successfully");
		} catch (Exception e) {
			telemetry.addData("Wheel Motors Status", "Initialization Failed: " + e.getMessage());
		}
	}
	
	// Expansion Hub Motors 초기화
	private void initializeExpansionHubMotors() {
		try {
			// Arm Motor 初期化
			armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");
			if (armMotor != null) {
				armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				telemetry.addData("Arm Motor", "Found");
			} else {
				telemetry.addData("Arm Motor", "Not Found");
			}
			
			liftMotor = hardwareMap.get(DcMotor.class, "liftmotor");
			
			// モーターのゼロパワー動作を設定
			liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			
			// Lift Motorの回転方向をFORWARDに設定
			liftMotor.setDirection(DcMotor.Direction.FORWARD);
			
			telemetry.addData("Expansion Hub Motors Status", "Initialized Successfully");
			telemetry.addData("Lift Motor", liftMotor != null ? "Found - FORWARD direction" : "Not Found");
		} catch (Exception e) {
			telemetry.addData("Expansion Hub Motors Status", "Initialization Failed: " + e.getMessage());
			telemetry.addData("Error Details", e.toString());
		}
	}
	
	// HD Hex Motor 제어
	private void controlCoreHexMotor(float rightTrigger) {
		if (coreHexMotor == null) return;
		if (rightTrigger < 0.1) {
			coreHexMotor.setPower(0);
		} else {
			coreHexMotor.setPower(rightTrigger);
		}
	}
	
	// Wheel Motors 제어
	private void controlWheelMotors(float rightY, boolean leftBumper, boolean rightBumper) {
		if (leftMotor != null && rightMotor != null) {
			float leftPower = 0.0f;
			float rightPower = 0.0f;
			
			// 前後運動制御 (Right Joystick Y-axis) - 逆方向
			if (Math.abs(rightY) >= 0.1) {
				leftPower = -rightY;  // 逆方向
				rightPower = -rightY; // 逆方向
			}
			
			// 回転制御 (L1/R1 buttons)
			if (leftBumper) {
				// 右回転 (L1)
				leftPower += 0.5f;
				rightPower -= 0.5f;
			}
			
			if (rightBumper) {
				// 左回転 (R1)
				leftPower -= 0.5f;
				rightPower += 0.5f;
			}
			
			// パワー値を-1.0から1.0の範囲に制限
			leftPower = Math.max(-1.0f, Math.min(1.0f, leftPower));
			rightPower = Math.max(-1.0f, Math.min(1.0f, rightPower));
			
			// モーターにパワーを設定
			leftMotor.setPower(leftPower);
			rightMotor.setPower(rightPower);
		}
	}
	
	// Expansion Hub Motors 제어 (L2 trigger + Left Joystick Y-axis)
	private void controlExpansionHubMotors(float leftTrigger, float leftJoystickY) {
		// Arm Motor 제어 (L2 trigger)
		if (armMotor != null) {
			// Dead zone 적용 (0.1 이하의 값은 무시)
			if (leftTrigger < 0.1) {
				armMotor.setPower(0.0);
			} else {
				// L2 trigger 값을 모터 파워로 사용 (0.0 ~ 1.0)
				armMotor.setPower(leftTrigger);
			}
		} else {
			telemetry.addData("DEBUG", "Arm Motor is null");
		}
		
		// Lift Motor 제어 (Left Joystick Y-axis)
		if (liftMotor != null) {
			// Dead zone 적용 (0.1 이하의 값은 무시)
			if (Math.abs(leftJoystickY) < 0.1) {
				liftMotor.setPower(0.0);
			} else {
				// Left Joystick Y-axis 값을 모터 파워로 사용 (-1.0 ~ 1.0)
				liftMotor.setPower(leftJoystickY);
			}
		} else {
			telemetry.addData("DEBUG", "Lift Motor is null");
		}
		
		// デバッグ情報を追加
		telemetry.addData("DEBUG", "L2: %.2f, Left Joystick Y: %.2f", leftTrigger, leftJoystickY);
	}
	
	// HTTP 서버 시작
	private void startRobotControlServer() {
		serverRunning.set(true);
		new Thread(() -> {
			try {
				serverSocket = new ServerSocket(robotControlPort);
				telemetry.addData("Robot Control Server", "Started on port " + robotControlPort);
				telemetry.update();
				
				while (serverRunning.get() && !Thread.currentThread().isInterrupted()) {
					try {
						Socket clientSocket = serverSocket.accept();
						handleClientConnection(clientSocket);
					} catch (Exception e) {
						if (serverRunning.get()) {
							robotControlLastError = "Server Error: " + e.getMessage();
						}
					}
				}
			} catch (Exception e) {
				robotControlLastError = "Server Start Error: " + e.getMessage();
			}
		}).start();
	}
	
	// HTTP 서버 정리
	private void stopRobotControlServer() {
		serverRunning.set(false);
		if (serverSocket != null && !serverSocket.isClosed()) {
			try {
				serverSocket.close();
			} catch (Exception e) {
				// Ignore
			}
		}
	}
	
	// 클라이언트 연결 처리
	private void handleClientConnection(Socket clientSocket) {
		new Thread(() -> {
			try (BufferedReader in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
				 PrintWriter out = new PrintWriter(clientSocket.getOutputStream(), true)) {
				
				// HTTP 요청 읽기
				String requestLine = in.readLine();
				telemetry.addData("DEBUG", "HTTP Request: " + requestLine);
				
				if (requestLine != null && requestLine.startsWith("POST")) {
					
					// 헤더 읽기
					String line;
					int contentLength = 0;
					while ((line = in.readLine()) != null && !line.isEmpty()) {
						if (line.toLowerCase().startsWith("content-length:")) {
							contentLength = Integer.parseInt(line.split(":")[1].trim());
							telemetry.addData("DEBUG", "Content-Length: " + contentLength);
						}
					}
					
					// JSON 본문 읽기
					StringBuilder jsonBody = new StringBuilder();
					for (int i = 0; i < contentLength; i++) {
						int c = in.read();
						if (c == -1) break;
						jsonBody.append((char) c);
					}
					
					// JSON 파싱 및 명령 처리
					String json = jsonBody.toString();
					telemetry.addData("DEBUG", "Received JSON: " + json.substring(0, Math.min(100, json.length())));
					if (json.contains("motorPower")) {
						// 모터 제어 명령 처리
						int start = json.indexOf("motorPower") + 12;
						int end = json.indexOf("}", start);
						if (end == -1) end = json.length();
						
						String motorPowerStr = json.substring(start, end).replaceAll("[^0-9.-]", "");
						float motorPower = Float.parseFloat(motorPowerStr);
						
						// 모터 파워 설정
						receivedMotorPower.set(motorPower);
						robotControlReceiveCount++;
						robotControlSuccessCount++;
						
						// HTTP 응답
						out.println("HTTP/1.1 200 OK");
						out.println("Content-Type: application/json");
						out.println();
						out.println("{\"status\":\"success\",\"motorPower\":" + motorPower + "}");
					} else if (json.contains("fileName")) {
						// 파일 정보 처리
						processFileInfo(json);
						robotControlReceiveCount++;
						robotControlSuccessCount++;
						
						// HTTP 응답
						out.println("HTTP/1.1 200 OK");
						out.println("Content-Type: application/json");
						out.println();
						out.println("{\"status\":\"success\",\"fileInfo\":\"received\"}");
					} else {
						robotControlErrorCount++;
						out.println("HTTP/1.1 400 Bad Request");
						out.println("Content-Type: application/json");
						out.println();
						out.println("{\"status\":\"error\",\"message\":\"Invalid JSON format\"}");
					}
				} else {
					out.println("HTTP/1.1 405 Method Not Allowed");
					out.println("Content-Type: application/json");
					out.println();
					out.println("{\"status\":\"error\",\"message\":\"Only POST method allowed\"}");
				}
				
			} catch (Exception e) {
				robotControlErrorCount++;
				robotControlLastError = "Client Error: " + e.getMessage();
			} finally {
				try {
					clientSocket.close();
				} catch (Exception e) {
					// Ignore
				}
			}
		}).start();
	}
	
	// 파일 정보 처리
	private void processFileInfo(String json) {
		try {
			telemetry.addData("DEBUG", "Processing file info JSON: " + json.substring(0, Math.min(100, json.length())));
			
			// 간단한 JSON 파싱
			if (json.contains("fileName")) {
				int nameStart = json.indexOf("fileName") + 11;
				int nameEnd = json.indexOf("\"", nameStart);
				lastFileName = json.substring(nameStart, nameEnd);
				telemetry.addData("DEBUG", "Parsed fileName: " + lastFileName);
			}
			
			if (json.contains("frameCount")) {
				int countStart = json.indexOf("frameCount") + 12;
				int countEnd = json.indexOf(",", countStart);
				if (countEnd == -1) countEnd = json.indexOf("}", countStart);
				lastFrameCount = Integer.parseInt(json.substring(countStart, countEnd));
				telemetry.addData("DEBUG", "Parsed frameCount: " + lastFrameCount);
			}
			
			if (json.contains("duration")) {
				int durationStart = json.indexOf("duration") + 10;
				int durationEnd = json.indexOf(",", durationStart);
				if (durationEnd == -1) durationEnd = json.indexOf("}", durationStart);
				lastDuration = Double.parseDouble(json.substring(durationStart, durationEnd));
				telemetry.addData("DEBUG", "Parsed duration: " + lastDuration);
			}
			
			if (json.contains("fileSize")) {
				int sizeStart = json.indexOf("fileSize") + 10;
				int sizeEnd = json.indexOf(",", sizeStart);
				if (sizeEnd == -1) sizeEnd = json.indexOf("}", sizeStart);
				lastFileSize = Long.parseLong(json.substring(sizeStart, sizeEnd));
				telemetry.addData("DEBUG", "Parsed fileSize: " + lastFileSize);
			}
			
			if (json.contains("uploadTime")) {
				int timeStart = json.indexOf("uploadTime") + 12;
				int timeEnd = json.indexOf("\"", timeStart);
				lastUploadTime = json.substring(timeStart, timeEnd);
				telemetry.addData("DEBUG", "Parsed uploadTime: " + lastUploadTime);
			}
			
			fileInfoStatus = "File received successfully";
			telemetry.addData("DEBUG", "File info processing completed successfully");
			
		} catch (Exception e) {
			fileInfoStatus = "Error parsing file info: " + e.getMessage();
			telemetry.addData("DEBUG", "File info parsing error: " + e.getMessage());
		}
	}
	
	// ロボット制御コマンドの確認
	private void checkRobotControlCommands() {
		// HTTP 서버에서 받은 모터 제어 명령 처리
		Float motorPower = receivedMotorPower.getAndSet(0.0f);
		if (motorPower != 0.0f && coreHexMotor != null) {
			// Dead zone 적용
			if (Math.abs(motorPower) < 0.1) {
				coreHexMotor.setPower(0.0);
			} else {
				coreHexMotor.setPower(motorPower);
			}
		}
	}
	
	// Distance Sensorから距離を取得
	private double getDistance() {
		try {
			if (distanceSensor != null) {
				double distance = distanceSensor.getDistance(DistanceUnit.CM);
				telemetry.addData("DEBUG", "Distance read successfully: %.2f cm", distance);
				return distance;
			} else {
				telemetry.addData("DEBUG", "Distance Sensor is null - cannot read distance");
				return -1.0; // センサーが初期化されていない場合
			}
		} catch (Exception e) {
			telemetry.addData("DEBUG", "Distance read exception: " + e.toString());
			distanceLastError = "Distance Read Error: " + e.getMessage();
			return -1.0;
		}
	}
	
	// Distance Sensorの警告レベルを取得
	private String getDistanceWarningLevel(double distanceCm) {
		if (distanceCm < 0) {
			return "ERROR"; // センサーエラー
		} else if (distanceCm < CRITICAL_DISTANCE_CM) {
			return "CRITICAL"; // 危険レベル
		} else if (distanceCm < WARNING_DISTANCE_CM) {
			return "WARNING"; // 警告レベル
		} else {
			return "SAFE"; // 安全レベル
		}
	}

	// IMU 초기화 (コメントアウト)
	// private void initializeIMU() {
	//	 try {
	//		 imu = hardwareMap.get(IMU.class, "imu");
	//		 IMU.Parameters parameters = new IMU.Parameters();
	//		 imu.initialize(parameters);
	//		 telemetry.addData("IMU Status", "Initialized Successfully");
	//	 } catch (Exception e) {
	//		 telemetry.addData("IMU Status", "Initialization Failed: " + e.getMessage());
	//		 imuLastError = "IMU Init Error: " + e.getMessage();
	//	 }
	// }

	// 記録開始 (Webダッシュボードで制御するため削除)
	// private void startRecording() {
	//	 isRecording = true;
	//	 recordedFrames.clear();
	//	 recordStartTime = System.currentTimeMillis();
	//	 telemetry.addData("Recording", "Started - Press A to stop");
	//	 telemetry.update();
	// }
	
	// 記録停止 (Webダッシュボードで制御するため削除)
	// private void stopRecording() {
	//	 isRecording = false;
	//	 long recordDuration = System.currentTimeMillis() - recordStartTime;
	//	 telemetry.addData("Recording", "Stopped - %d frames, %.2f seconds", 
	//					  recordedFrames.size(), recordDuration / 1000.0);
	//	 telemetry.update();
	//	 
	//	 // 記録データをサーバーに送信
	//	 sendRecordedData();
	// }
	
	// フレーム記録 (Webダッシュボードで処理するため削除)
	// private void recordFrame(float leftX1, float leftY1, float rightX1, float rightY1,
	//						float leftX2, float leftY2, float rightX2, float rightY2) {
	//	 long timestamp = System.currentTimeMillis() - recordStartTime;
	//	 JoystickFrame frame = new JoystickFrame(timestamp, leftX1, leftY1, rightX1, rightY1,
	//											leftX2, leftY2, rightX2, rightY2);
	//	 recordedFrames.add(frame);
	// }
	
	// 記録データ送信 (Webダッシュボードで処理するため削除)
	// private void sendRecordedData() {
	//	 if (recordedFrames.isEmpty()) return;
	//	 
	//	 new Thread(() -> {
	//		 try {
	//			 // JSONデータを作成
	//			 StringBuilder json = new StringBuilder();
	//			 json.append("{\"frames\":[");
	//			 
	//			 for (int i = 0; i < recordedFrames.size(); i++) {
	//				 JoystickFrame frame = recordedFrames.get(i);
	//				 if (i > 0) json.append(",");
	//				 json.append(String.format(
	//					 "{\"timestamp\":%d,\"leftX1\":%.3f,\"leftY1\":%.3f,\"rightX1\":%.3f,\"rightY1\":%.3f," +
	//					 "\"leftX2\":%.3f,\"leftY2\":%.3f,\"rightX2\":%.3f,\"rightY2\":%.3f}",
	//					 frame.timestamp, frame.leftX1, frame.leftY1, frame.rightX1, frame.rightY1,
	//					 frame.leftX2, frame.leftY2, frame.rightX2, frame.rightY2
	//				 ));
	//			 }
	//			 json.append("]}");
	//			 
	//			 URL url = new URL(SERVER_URL + RECORD_ENDPOINT);
	//			 HttpURLConnection conn = (HttpURLConnection) url.openConnection();
	//			 conn.setRequestMethod("POST");
	//			 conn.setRequestProperty("Content-Type", "application/json");
	//			 conn.setDoOutput(true);
	//			 conn.setConnectTimeout(10000);
	//			 conn.setReadTimeout(10000);
	//			 
	//			 OutputStream os = conn.getOutputStream();
	//			 os.write(json.toString().getBytes("UTF-8"));
	//			 os.flush();
	//			 os.close();
	//			 
	//			 int responseCode = conn.getResponseCode();
	//			 telemetry.addData("Record Send", "Response: " + responseCode);
	//			 conn.disconnect();
	//			 
	//		 } catch (Exception e) {
	//			 telemetry.addData("Record Send Error", e.getMessage());
	//		 }
	//	 }).start();
	// }
	
	// 再生開始 (Webダッシュボードで制御するため削除)
	// private void startPlayback() {
	//	 if (recordedFrames.isEmpty()) return;
	//	 
	//	 new Thread(() -> {
	//		 try {
	//			 URL url = new URL(SERVER_URL + PLAYBACK_ENDPOINT);
	//			 HttpURLConnection conn = (HttpURLConnection) url.openConnection();
	//			 conn.setRequestMethod("POST");
	//			 conn.setRequestProperty("Content-Type", "application/json");
	//			 conn.setDoOutput(true);
	//			 conn.setConnectTimeout(10000);
	//			 conn.setReadTimeout(10000);
	//			 
	//			 String json = "{\"action\":\"start\"}";
	//			 OutputStream os = conn.getOutputStream();
	//			 os.write(json.getBytes("UTF-8"));
	//			 os.flush();
	//			 os.close();
	//			 
	//			 int responseCode = conn.getResponseCode();
	//			 telemetry.addData("Playback Start", "Response: " + responseCode);
	//			 conn.disconnect();
	//			 
	//		 } catch (Exception e) {
	//			 telemetry.addData("Playback Error", e.getMessage());
	//		 }
	//	 }).start();
	// }

	private void sendJoystick1Data(float leftX, float leftY, float rightX, float rightY) {
		String json = String.format(
			"{\"leftX\":%.2f, \"leftY\":%.2f, \"rightX\":%.2f, \"rightY\":%.2f}",
			leftX, leftY, rightX, rightY
		);

		new Thread(() -> {
			try {
				URL url = new URL(SERVER_URL + JOYSTICK1_ENDPOINT);
				HttpURLConnection conn = (HttpURLConnection) url.openConnection();
				conn.setRequestMethod("POST");
				conn.setRequestProperty("Content-Type", "application/json");
				conn.setDoOutput(true);
				conn.setConnectTimeout(10000);
				conn.setReadTimeout(10000);
				
				OutputStream os = conn.getOutputStream();
				os.write(json.getBytes("UTF-8"));
				os.flush();
				os.close();

				int responseCode = conn.getResponseCode();
				
				if (responseCode == 200) {
					joystick1SuccessCount++;
					joystick1LastError = "";
				} else {
					joystick1ErrorCount++;
					joystick1LastError = "HTTP " + responseCode;
				}
				conn.disconnect();
				
			} catch (Exception e) {
				joystick1ErrorCount++;
				joystick1LastError = e.getClass().getSimpleName() + ": " + e.getMessage();
			}
		}).start();
	}

	private void sendJoystick2Data(float leftX, float leftY, float rightX, float rightY) {
		String json = String.format(
			"{\"leftX\":%.2f, \"leftY\":%.2f, \"rightX\":%.2f, \"rightY\":%.2f}",
			leftX, leftY, rightX, rightY
		);

		new Thread(() -> {
			try {
				URL url = new URL(SERVER_URL + JOYSTICK2_ENDPOINT);
				HttpURLConnection conn = (HttpURLConnection) url.openConnection();
				conn.setRequestMethod("POST");
				conn.setRequestProperty("Content-Type", "application/json");
				conn.setDoOutput(true);
				conn.setConnectTimeout(10000);
				conn.setReadTimeout(10000);
				
				OutputStream os = conn.getOutputStream();
				os.write(json.getBytes("UTF-8"));
				os.flush();
				os.close();

				int responseCode = conn.getResponseCode();
				
				if (responseCode == 200) {
					joystick2SuccessCount++;
					joystick2LastError = "";
				} else {
					joystick2ErrorCount++;
					joystick2LastError = "HTTP " + responseCode;
				}
				conn.disconnect();
				
			} catch (Exception e) {
				joystick2ErrorCount++;
				joystick2LastError = e.getClass().getSimpleName() + ": " + e.getMessage();
			}
		}).start();
	}
	
	private void sendDistanceData(double distanceCm) {
		String warningLevel = getDistanceWarningLevel(distanceCm);
		String json = String.format(
			"{\"distance\":%.2f, \"warningLevel\":\"%s\", \"warningDistance\":%.1f, \"criticalDistance\":%.1f}",
			distanceCm, warningLevel, WARNING_DISTANCE_CM, CRITICAL_DISTANCE_CM
		);

		new Thread(() -> {
			try {
				URL url = new URL(SERVER_URL + DISTANCE_ENDPOINT);
				HttpURLConnection conn = (HttpURLConnection) url.openConnection();
				conn.setRequestMethod("POST");
				conn.setRequestProperty("Content-Type", "application/json");
				conn.setDoOutput(true);
				conn.setConnectTimeout(10000);
				conn.setReadTimeout(10000);
				
				OutputStream os = conn.getOutputStream();
				os.write(json.getBytes("UTF-8"));
				os.flush();
				os.close();

				int responseCode = conn.getResponseCode();
				
				if (responseCode == 200) {
					distanceSuccessCount++;
					distanceLastError = "";
				} else {
					distanceErrorCount++;
					distanceLastError = "HTTP " + responseCode;
				}
				conn.disconnect();
				
			} catch (Exception e) {
				distanceErrorCount++;
				distanceLastError = e.getClass().getSimpleName() + ": " + e.getMessage();
			}
		}).start();
	}
	
	private void sendColorSensorData() {
		if (colorSensor == null) {
			colorErrorCount++;
			colorLastError = "Color Sensor not initialized";
			return;
		}
		
		try {
			// Color Sensor 데이터 수집
			int red = colorSensor.red();
			int green = colorSensor.green();
			int blue = colorSensor.blue();
			int alpha = colorSensor.alpha();
			
			// HSV 값 계산
			float[] hsv = new float[3];
			android.graphics.Color.RGBToHSV(red, green, blue, hsv);
			float hue = hsv[0];
			float saturation = hsv[1];
			float value = hsv[2];
			
			// 색상 이름 결정
			String colorName = getColorName(hue, saturation, value);
			
			String json = String.format(
				"{\"red\":%d,\"green\":%d,\"blue\":%d,\"alpha\":%d,\"hue\":%.2f,\"saturation\":%.2f,\"value\":%.2f,\"colorName\":\"%s\"}",
				red, green, blue, alpha, hue, saturation, value, colorName
			);

			new Thread(() -> {
				try {
					URL url = new URL(SERVER_URL + COLOR_ENDPOINT);
					HttpURLConnection conn = (HttpURLConnection) url.openConnection();
					conn.setRequestMethod("POST");
					conn.setRequestProperty("Content-Type", "application/json");
					conn.setDoOutput(true);
					conn.setConnectTimeout(10000);
					conn.setReadTimeout(10000);
					
					OutputStream os = conn.getOutputStream();
					os.write(json.getBytes("UTF-8"));
					os.flush();
					os.close();

					int responseCode = conn.getResponseCode();
					
					if (responseCode == 200) {
						colorSuccessCount++;
						colorLastError = "";
					} else {
						colorErrorCount++;
						colorLastError = "HTTP " + responseCode;
					}
					conn.disconnect();
					
				} catch (Exception e) {
					colorErrorCount++;
					colorLastError = e.getClass().getSimpleName() + ": " + e.getMessage();
				}
			}).start();
			
		} catch (Exception e) {
			colorErrorCount++;
			colorLastError = "Color Sensor Read Error: " + e.getMessage();
		}
	}
	
	// 색상 이름 감지 (HSV 값 기반)
	private String getColorName(float hue, float saturation, float value) {
		// 색상이 너무 어두운 경우 (검정)
		if (value < 0.1) {
			return "Black";
		}
		
		// 색상이 너무 밝은 경우 (흰색/회색)
		if (saturation < 0.1) {
			if (value > 0.8) {
				return "White";
			} else if (value > 0.3) {
				return "Gray";
			} else {
				return "Dark Gray";
			}
		}

		// 색조 기반 색상 감지
		if (hue < 15 || hue >= 345) {
			return "Red";
		} else if (hue < 45) {
			return "Orange";
		} else if (hue < 75) {
			return "Yellow";
		} else if (hue < 165) {
			return "Green";
		} else if (hue < 195) {
			return "Cyan";
		} else if (hue < 255) {
			return "Blue";
		} else if (hue < 285) {
			return "Magenta";
		} else if (hue < 315) {
			return "Pink";
		} else {
			return "Red";
		}
	}

	// IMUデータ送信 (コメントアウト)
	// private void sendIMUData() {
	//	 try {
	//		 // SimpleIMUTest.java와 동일한 방식으로 IMU 데이터 수집
	//		 YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
	//		 AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
	//		 
	//		 // JSON 데이터 생성
	//		 String json = String.format(
	//			 "{\"yaw\":%.2f, \"pitch\":%.2f, \"roll\":%.2f, \"angularVelocityZ\":%.2f}",
	//			 orientation.getYaw(AngleUnit.DEGREES),
	//			 orientation.getPitch(AngleUnit.DEGREES),
	//			 orientation.getRoll(AngleUnit.DEGREES),
	//			 angularVelocity.zRotationRate
	//		 );

	//		 // HTTP POST를 별도 스레드로 송신
	//		 new Thread(() -> {
	//			 try {
	//				 URL url = new URL(SERVER_URL + IMU_ENDPOINT);
	//				 HttpURLConnection conn = (HttpURLConnection) url.openConnection();
	//				 conn.setRequestMethod("POST");
	//				 conn.setRequestProperty("Content-Type", "application/json");
	//				 conn.setDoOutput(true);
	//				 conn.setConnectTimeout(10000);
	//				 conn.setReadTimeout(10000);
	   
	//				 OutputStream os = conn.getOutputStream();
	//				 os.write(json.getBytes("UTF-8"));
	//				 os.flush();
	//				 os.close();
	   
	//				 int responseCode = conn.getResponseCode();
	//				 
	//				 if (responseCode == 200) {
	//					 imuSuccessCount++;
	//					 imuLastError = "";
	//				 } else {
	//					 imuErrorCount++;
	//					 imuLastError = "HTTP " + responseCode;
	//				 }
	//				 
	//				 conn.disconnect();
	//			 } catch (Exception e) {
	//				 imuErrorCount++;
	//				 imuLastError = e.getClass().getSimpleName() + ": " + e.getMessage();
	//			 }
	//		 }).start();
	//		 
	//	 } catch (Exception e) {
	//		 imuErrorCount++;
	//		 imuLastError = "IMU Data Error: " + e.getMessage();
	//	 }
	// }

	private void displayTelemetry(float leftX1, float leftY1, float rightX1, float rightY1, 
								 float leftX2, float leftY2, float rightX2, float rightY2,
								 double distanceCm) {
		// Joystick 1 섹션
		telemetry.addData("=== Joystick 1 (Gamepad 1) ===", "");
		telemetry.addData("Joystick 1 Values", "L(%.2f,%.2f) R(%.2f,%.2f)", leftX1, leftY1, rightX1, rightY1);
		telemetry.addData("Send Count 1", joystick1SendCount);
		telemetry.addData("Success Count 1", joystick1SuccessCount);
		telemetry.addData("Error Count 1", joystick1ErrorCount);
		if (!joystick1LastError.isEmpty()) {
			telemetry.addData("Last Error 1", joystick1LastError);
		}
		
		// Joystick 2 섹션
		telemetry.addData("=== Joystick 2 (Gamepad 2) ===", "");
		telemetry.addData("Joystick 2 Values", "L(%.2f,%.2f) R(%.2f,%.2f)", leftX2, leftY2, rightX2, rightY2);
		telemetry.addData("Send Count 2", joystick2SendCount);
		telemetry.addData("Success Count 2", joystick2SuccessCount);
		telemetry.addData("Error Count 2", joystick2ErrorCount);
		if (!joystick2LastError.isEmpty()) {
			telemetry.addData("Last Error 2", joystick2LastError);
		}
		
		// HD Hex Motor 섹션
		telemetry.addData("=== HD Hex Motor ===", "");
		if (coreHexMotor != null) {
			telemetry.addData("Motor Power", "%.2f", coreHexMotor.getPower());
			telemetry.addData("Motor Direction", "REVERSE");
			telemetry.addData("Control", "R2 Trigger (Gamepad 1)");
			telemetry.addData("Forward", "Press R2 trigger");
			telemetry.addData("Stop", "Release R2 trigger");
			telemetry.addData("Robot Control", "Ready to receive commands from web");
		} else {
			telemetry.addData("Motor Status", "Not initialized");
		}
		
		// Expansion Hub Motors 섹션
		telemetry.addData("=== Expansion Hub Motors ===", "");
		if (armMotor != null) {
			telemetry.addData("Arm Motor Power", "%.2f", armMotor.getPower());
			telemetry.addData("Arm Control", "L2 Trigger (Gamepad 1)");
			telemetry.addData("Arm Operation", "Press L2 to control arm motor");
			telemetry.addData("Arm Motor Status", "Connected");
		} else {
			telemetry.addData("Arm Motor Status", "Not initialized - Check hardware configuration");
			telemetry.addData("Expected Name", "armmotor");
		}
		
		if (liftMotor != null) {
			telemetry.addData("Lift Motor Power", "%.2f", liftMotor.getPower());
			telemetry.addData("Lift Motor Direction", "FORWARD");
			telemetry.addData("Lift Control", "Left Joystick Y-axis (Gamepad 1)");
			telemetry.addData("Lift Operation", "Push up/down on left stick to control lift motor");
			telemetry.addData("Lift Motor Status", "Connected - FORWARD direction");
		} else {
			telemetry.addData("Lift Motor Status", "Not initialized - Check hardware configuration");
			telemetry.addData("Expected Name", "liftmotor");
		}
		
		// Robot Control Server 섹션
		telemetry.addData("=== Robot Control Server ===", "");
		telemetry.addData("Server Status", serverRunning.get() ? "Running" : "Stopped");
		telemetry.addData("Server Port", robotControlPort);
		telemetry.addData("Receive Count", robotControlReceiveCount);
		telemetry.addData("Success Count", robotControlSuccessCount);
		telemetry.addData("Error Count", robotControlErrorCount);
		if (!robotControlLastError.isEmpty()) {
			telemetry.addData("Last Error", robotControlLastError);
		}
		
		// File Upload 섹션
		telemetry.addData("=== File Upload Info ===", "");
		telemetry.addData("Status", fileInfoStatus);
		if (!lastFileName.isEmpty()) {
			telemetry.addData("File Name", lastFileName);
			telemetry.addData("Frame Count", lastFrameCount);
			telemetry.addData("Duration", "%.2f seconds", lastDuration);
			telemetry.addData("File Size", "%d bytes", lastFileSize);
			telemetry.addData("Upload Time", lastUploadTime);
		}
		
		// Distance Sensor 섹션
		telemetry.addData("=== Distance Sensor ===", "");
		if (distanceCm >= 0) {
			String warningLevel = getDistanceWarningLevel(distanceCm);
			telemetry.addData("Distance", "%.2f cm", distanceCm);
			telemetry.addData("Warning Level", warningLevel);
			
			// 警告レベルに応じて色分け表示
			if (warningLevel.equals("CRITICAL")) {
				telemetry.addData("⚠️ CRITICAL", "STOP! Too close to obstacle!");
			} else if (warningLevel.equals("WARNING")) {
				telemetry.addData("⚠️ WARNING", "Approaching obstacle!");
			} else {
				telemetry.addData("✅ SAFE", "Safe distance maintained");
			}
		} else {
			telemetry.addData("Distance", "ERROR - Sensor not available");
		}
		
		telemetry.addData("Distance Send Count", distanceSendCount);
		telemetry.addData("Distance Success Count", distanceSuccessCount);
		telemetry.addData("Distance Error Count", distanceErrorCount);
		if (!distanceLastError.isEmpty()) {
			telemetry.addData("Distance Last Error", distanceLastError);
		}
		
		// Color Sensor 섹션
		telemetry.addData("=== Color Sensor ===", "");
		if (colorSensor != null) {
			try {
				int red = colorSensor.red();
				int green = colorSensor.green();
				int blue = colorSensor.blue();
				int alpha = colorSensor.alpha();
				
				// HSV 값
				float[] hsv = new float[3];
				android.graphics.Color.RGBToHSV(red, green, blue, hsv);
				float hue = hsv[0];
				float saturation = hsv[1];
				float value = hsv[2];
				
				// 색상 이름
				String colorName = getColorName(hue, saturation, value);
				
				telemetry.addData("Color Sensor", "R:%d G:%d B:%d A:%d", red, green, blue, alpha);
				telemetry.addData("HSV", "H:%.1f° S:%.1f%% V:%.1f%%", hue, saturation*100, value*100);
				telemetry.addData("Color Name", colorName);
				
			} catch (Exception e) {
				telemetry.addData("Color Sensor", "Error: " + e.getMessage());
			}
		} else {
			telemetry.addData("Color Sensor", "Not connected");
		}
		
		telemetry.addData("Color Send Count", colorSendCount);
		telemetry.addData("Color Success Count", colorSuccessCount);
		telemetry.addData("Color Error Count", colorErrorCount);
		if (!colorLastError.isEmpty()) {
			telemetry.addData("Color Last Error", colorLastError);
		}
		
		// 記録・再生 섹션 (Webダッシュボードで制御するため簡素化)
		telemetry.addData("=== Recording & Playback ===", "");
		telemetry.addData("Recording Control", "Use Web Dashboard");
		telemetry.addData("Web URL", "http://192.168.43.48:5000");
		telemetry.addData("Status", "Sending joystick & distance data continuously");
		telemetry.addData("Note", "Recording/Playback controlled from web");
		telemetry.addData("File Upload", "Supports template file upload and playback");
		
		// IMU 섹션 (コメントアウト)
		// telemetry.addData("=== IMU Data ===", "");
		// try {
		//	 YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
		//	 AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
		//	 
		//	 telemetry.addData("IMU Values", "Yaw:%.2f° Pitch:%.2f° Roll:%.2f° AngVelZ:%.2f°/s",
		//		 orientation.getYaw(AngleUnit.DEGREES),
		//		 orientation.getPitch(AngleUnit.DEGREES),
		//		 orientation.getRoll(AngleUnit.DEGREES),
		//		 angularVelocity.zRotationRate
		//	 );
		// } catch (Exception e) {
		//	 telemetry.addData("IMU Values", "Error: " + e.getMessage());
		// }
		// 
		// telemetry.addData("IMU Send Count", imuSendCount);
		// telemetry.addData("IMU Success Count", imuSuccessCount);
		// telemetry.addData("IMU Error Count", imuErrorCount);
		// if (!imuLastError.isEmpty()) {
		//	 telemetry.addData("IMU Last Error", imuLastError);
		// }
		
		telemetry.update();
	}
}
*/