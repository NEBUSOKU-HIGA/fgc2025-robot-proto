package org.firstinspires.ftc.teamcode;

import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class A1Flask {

	// Flaskサーバーの設定 setup
	private static final String FLASK_SERVER_URL = "http://192.168.43.48:5000";
	private static final String JOYSTICK1_ENDPOINT = "/joystick1";
	private static final String JOYSTICK2_ENDPOINT = "/joystick2";
	
	// 送信カウンター
	private int sendCount = 0;
	private int successCount = 0;
	private int errorCount = 0;
	private String lastError = "";
	
	// 非同期実行用のExecutorService
	private ExecutorService executor;

	public A1Flask() {
		// 単一スレッドのExecutorServiceを作成
		executor = Executors.newSingleThreadExecutor();
	}

	// ジョイスティックデータを送信するメソッド（非同期）- Gamepad1用
	public void sendJoystick1Data(double leftStickX, double leftStickY, double rightStickX, double rightStickY,
								 double leftTrigger, double rightTrigger,
								 boolean leftBumper, boolean rightBumper,
								 boolean aButton, boolean bButton, boolean xButton, boolean yButton,
								 boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight) {
		
		// flask.pyが理解できる形式でJSONデータを作成
		String jsonData = createFlaskCompatibleJSON(leftStickX, leftStickY, rightStickX, rightStickY);

		// 非同期でFlaskサーバーのjoystick1エンドポイントに送信
		executor.submit(() -> sendToFlaskServer(jsonData, JOYSTICK1_ENDPOINT));
	}
	
	// ジョイスティックデータを送信するメソッド（非同期）- Gamepad2用
	public void sendJoystick2Data(double leftStickX, double leftStickY, double rightStickX, double rightStickY,
								 double leftTrigger, double rightTrigger,
								 boolean leftBumper, boolean rightBumper,
								 boolean aButton, boolean bButton, boolean xButton, boolean yButton,
								 boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight) {
		
		// flask.pyが理解できる形式でJSONデータを作成
		String jsonData = createFlaskCompatibleJSON(leftStickX, leftStickY, rightStickX, rightStickY);

		// 非同期でFlaskサーバーのjoystick2エンドポイントに送信
		executor.submit(() -> sendToFlaskServer(jsonData, JOYSTICK2_ENDPOINT));
	}
	
	// 後方互換性のためのメソッド（既存のA1Task.javaとの互換性を保つ）
	public void sendJoystickData(double leftStickX, double leftStickY, double rightStickX, double rightStickY,
								double leftTrigger, double rightTrigger,
								boolean leftBumper, boolean rightBumper,
								boolean aButton, boolean bButton, boolean xButton, boolean yButton,
								boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight) {
		
		// デフォルトでGamepad1として送信
		sendJoystick1Data(leftStickX, leftStickY, rightStickX, rightStickY,
						 leftTrigger, rightTrigger, leftBumper, rightBumper,
						 aButton, bButton, xButton, yButton,
						 dpadUp, dpadDown, dpadLeft, dpadRight);
	}

	// カラーセンサーデータを送信するメソッド（非同期）
	public void sendColorSensorData(int red, int green, int blue, int alpha, String colorName) {
		String jsonData = String.format(
			"{\"red\":%d,\"green\":%d,\"blue\":%d,\"alpha\":%d,\"colorName\":\"%s\"}",
			red, green, blue, alpha, colorName
		);
		
		// 非同期でFlaskサーバーのcolor_sensorエンドポイントに送信
		executor.submit(() -> sendToFlaskServer(jsonData, "/color_sensor"));
	}

	// ディスタンスセンサーデータを送信するメソッド（非同期）
	public void sendDistanceSensorData(double distanceCm, String warningLevel) {
		String jsonData = String.format(
			"{\"distanceCm\":%.1f,\"warningLevel\":\"%s\"}",
			distanceCm, warningLevel
		);
		
		// 非同期でFlaskサーバーのdistance_sensorエンドポイントに送信
		executor.submit(() -> sendToFlaskServer(jsonData, "/distance_sensor"));
	}

	// flask.pyが理解できる形式でJSONデータを作成
	private String createFlaskCompatibleJSON(double leftX, double leftY, double rightX, double rightY) {
		return String.format(
			"{\"leftX\": %.3f, \"leftY\": %.3f, \"rightX\": %.3f, \"rightY\": %.3f}",
			leftX, leftY, rightX, rightY
		);
	}
	
	// 元の詳細なJSONデータを作成（後方互換性のため）
	private String createJoystickJSON(double leftX, double leftY, double rightX, double rightY,
									double leftTrigger, double rightTrigger,
									boolean leftBumper, boolean rightBumper,
									boolean aButton, boolean bButton, boolean xButton, boolean yButton,
									boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight) {
		
		return String.format(
			"{\"joystick_data\": {" +
			"\"left_stick\": {\"x\": %.3f, \"y\": %.3f}," +
			"\"right_stick\": {\"x\": %.3f, \"y\": %.3f}," +
			"\"triggers\": {\"left\": %.3f, \"right\": %.3f}," +
			"\"bumpers\": {\"left\": %s, \"right\": %s}," +
			"\"buttons\": {\"a\": %s, \"b\": %s, \"x\": %s, \"y\": %s}," +
			"\"dpad\": {\"up\": %s, \"down\": %s, \"left\": %s, \"right\": %s}," +
			"\"timestamp\": %d" +
			"}}",
			leftX, leftY, rightX, rightY,
			leftTrigger, rightTrigger,
			leftBumper, rightBumper,
			aButton, bButton, xButton, yButton,
			dpadUp, dpadDown, dpadLeft, dpadRight,
			System.currentTimeMillis()
		);
	}

	// Flaskサーバーにデータを送信（タイムアウト付き）
	private void sendToFlaskServer(String jsonData, String endpoint) {
		sendCount++;
		
		try {
			URL url = new URL(FLASK_SERVER_URL + endpoint);
			HttpURLConnection connection = (HttpURLConnection) url.openConnection();
			
			// タイムアウト設定（3秒）
			connection.setConnectTimeout(3000);
			connection.setReadTimeout(3000);
			
			// HTTP POST設定
			connection.setRequestMethod("POST");
			connection.setRequestProperty("Content-Type", "application/json");
			connection.setRequestProperty("Accept", "application/json");
			connection.setDoOutput(true);
			
			// データを送信
			try (OutputStream os = connection.getOutputStream()) {
				byte[] input = jsonData.getBytes(StandardCharsets.UTF_8);
				os.write(input, 0, input.length);
			}
			
			// レスポンスを確認
			int responseCode = connection.getResponseCode();
			if (responseCode == HttpURLConnection.HTTP_OK) {
				successCount++;
				lastError = "";
				System.out.println("A1Flask: Successfully sent data to " + endpoint);
			} else {
				errorCount++;
				lastError = "HTTP Error: " + responseCode;
				System.out.println("A1Flask: HTTP Error " + responseCode + " when sending to " + endpoint);
			}
			
			connection.disconnect();
			
		} catch (Exception e) {
			errorCount++;
			lastError = "Connection Error: " + e.getMessage();
			System.out.println("A1Flask: Connection Error when sending to " + endpoint + ": " + e.getMessage());
		}
	}

	// 統計情報を取得するメソッド
	public int getSendCount() {
		return sendCount;
	}
	
	public int getSuccessCount() {
		return successCount;
	}
	
	public int getErrorCount() {
		return errorCount;
	}
	
	public String getLastError() {
		return lastError;
	}
	
	public double getSuccessRate() {
		return sendCount > 0 ? (double)successCount / sendCount * 100 : 0.0;
	}
	
	public String getServerURL() {
		return FLASK_SERVER_URL;
	}
	
	public String getJoystick1Endpoint() {
		return JOYSTICK1_ENDPOINT;
	}
	
	public String getJoystick2Endpoint() {
		return JOYSTICK2_ENDPOINT;
	}
	
	// リソースをクリーンアップ
	public void shutdown() {
		if (executor != null && !executor.isShutdown()) {
			executor.shutdown();
		}
	}
}
