package org.firstinspires.ftc.teamcode.network;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;

// Joystick 값을 Flask 서버로 송신하는 Task (Dual Joystick 지원)
public class SendJoystickTask implements Runnable {
    private final OpMode opMode;
    private static final String SERVER_URL = "http://192.168.43.48:5000";
    private static final String JOYSTICK1_ENDPOINT = "/joystick1";
    private static final String JOYSTICK2_ENDPOINT = "/joystick2";
    
    // 통계 변수들
    private static int sendCount1 = 0;
    private static int successCount1 = 0;
    private static int errorCount1 = 0;
    private static String lastError1 = "";
    
    private static int sendCount2 = 0;
    private static int successCount2 = 0;
    private static int errorCount2 = 0;
    private static String lastError2 = "";

    public SendJoystickTask(OpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void run() {
        // Gamepad 1 데이터 수집
        float leftX1 = opMode.gamepad1.left_stick_x;
        float leftY1 = -opMode.gamepad1.left_stick_y;
        float rightX1 = opMode.gamepad1.right_stick_x;
        float rightY1 = -opMode.gamepad1.right_stick_y;
        
        // Gamepad 2 데이터 수집
        float leftX2 = opMode.gamepad2.left_stick_x;
        float leftY2 = -opMode.gamepad2.left_stick_y;
        float rightX2 = opMode.gamepad2.right_stick_x;
        float rightY2 = -opMode.gamepad2.right_stick_y;

        // Joystick 1 데이터 전송
        String json1 = String.format(
            "{\"leftX\":%.2f, \"leftY\":%.2f, \"rightX\":%.2f, \"rightY\":%.2f}",
            leftX1, leftY1, rightX1, rightY1
        );
        sendJoystickData(json1, JOYSTICK1_ENDPOINT, 1);

        // Joystick 2 데이터 전송
        String json2 = String.format(
            "{\"leftX\":%.2f, \"leftY\":%.2f, \"rightX\":%.2f, \"rightY\":%.2f}",
            leftX2, leftY2, rightX2, rightY2
        );
        sendJoystickData(json2, JOYSTICK2_ENDPOINT, 2);
    }

    private void sendJoystickData(String json, String endpoint, int joystickNum) {
        // HTTP POST는 별도 스레드로 송신 (메인 루프 정지 방지)
        new Thread(() -> {
            try {
                URL url = new URL(SERVER_URL + endpoint);
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
                
                if (joystickNum == 1) {
                    sendCount1++;
                    if (responseCode == 200) {
                        successCount1++;
                        lastError1 = "";
                    } else {
                        errorCount1++;
                        lastError1 = "HTTP " + responseCode;
                    }
                } else {
                    sendCount2++;
                    if (responseCode == 200) {
                        successCount2++;
                        lastError2 = "";
                    } else {
                        errorCount2++;
                        lastError2 = "HTTP " + responseCode;
                    }
                }
                
                conn.disconnect();
            } catch (Exception e) {
                if (joystickNum == 1) {
                    errorCount1++;
                    lastError1 = e.getClass().getSimpleName() + ": " + e.getMessage();
                } else {
                    errorCount2++;
                    lastError2 = e.getClass().getSimpleName() + ": " + e.getMessage();
                }
            }
        }).start();
    }
    
    // 통계 정보 반환 메서드들
    public static int getSendCount1() { return sendCount1; }
    public static int getSuccessCount1() { return successCount1; }
    public static int getErrorCount1() { return errorCount1; }
    public static String getLastError1() { return lastError1; }
    
    public static int getSendCount2() { return sendCount2; }
    public static int getSuccessCount2() { return successCount2; }
    public static int getErrorCount2() { return errorCount2; }
    public static String getLastError2() { return lastError2; }
}
