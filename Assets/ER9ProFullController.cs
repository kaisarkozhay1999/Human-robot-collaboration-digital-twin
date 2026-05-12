using UnityEngine;
using TMPro;
using System.Text;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;

public class ER9ProFullController : MonoBehaviour
{
    [Header("Robot Joints")]
    public Transform joint1;
    public Transform joint2;
    public Transform joint3;
    public Transform joint4;
    public Transform joint5;
    public Transform gripper;

    [Header("UI Text")]
    public TMP_Text baseValue;
    public TMP_Text shoulderValue;
    public TMP_Text elbowValue;
    public TMP_Text pitchValue;
    public TMP_Text rollValue;

    [Header("Settings")]
    public float moveSpeed = 5f;
    public float manualSpeed = 60f;

    [Header("MQTT")]
    public string brokerIP = "192.168.0.190";
    public string stateTopic = "robot/state";
    public string controlTopic = "robot/control";

    MqttClient client;

    float a1, a2, a3, a4, a5, g;
    float targetA1, targetA2, targetA3, targetA4, targetA5, targetG;

    Quaternion j1Start, j2Start, j3Start, j4Start, j5Start, gStart;

    string currentCommand = "";
    bool isHolding = false;
    bool manualControl = false;
    float lastSendTime = 0;

    float offsetJ1 = 0f;
    float offsetJ2 = -90f;
    float offsetJ3 = 10f;
    float offsetJ4 = 70f;
    float offsetJ5 = 0f;

    float J1_MIN = -138f, J1_MAX = 138f;
    float J2_MIN = -100f, J2_MAX = 100f;
    float J3_MIN = -107f, J3_MAX = 107f;
    float J4_MIN = -101f, J4_MAX = 101f;
    float J5_MIN = -368f, J5_MAX = 368f;

    public float CurrentA1 => a1;
    public float CurrentA2 => a2;
    public float CurrentA3 => a3;
    public float CurrentA4 => a4;
    public float CurrentA5 => a5;

    void Start()
    {
        j1Start = joint1.localRotation;
        j2Start = joint2.localRotation;
        j3Start = joint3.localRotation * Quaternion.Euler(0f, -90f, 0f);
        j4Start = joint4.localRotation * Quaternion.Euler(0f, -50f, 0f);
        j5Start = joint5.localRotation * Quaternion.Euler(0f, -90f, 0f);;
        gStart  = gripper.localRotation;

        ConnectMQTT();
        ResetRobot();
    }

    void ConnectMQTT()
    {
        try
        {
            client = new MqttClient(brokerIP);
            client.MqttMsgPublishReceived += OnMessageReceived;

            string clientId = System.Guid.NewGuid().ToString();
            client.Connect(clientId);

            client.Subscribe(new string[] { stateTopic }, new byte[] { 0 });

            Debug.Log("MQTT Connected");
        }
        catch (System.Exception e)
        {
            Debug.Log("MQTT ERROR: " + e.Message);
        }
    }

    void SendCommand(string msg)
    {
        if (Time.time - lastSendTime < 0.1f) return;

        if (client != null && client.IsConnected)
        {
            client.Publish(controlTopic, Encoding.UTF8.GetBytes(msg));
            lastSendTime = Time.time;
        }
    }

    void OnMessageReceived(object sender, MqttMsgPublishEventArgs e)
    {
        string msg = Encoding.UTF8.GetString(e.Message);

        string[] values = msg.Split(',');

        if (values.Length >= 6)
        {
            targetA1 = float.Parse(values[0]);
            targetA2 = float.Parse(values[1]);
            targetA3 = float.Parse(values[2]);
            targetA4 = float.Parse(values[3]);
            targetA5 = float.Parse(values[4]);
            targetG  = float.Parse(values[5]);
        }
    }

    void Update()
    {
        //  Continuous command sending
        if (isHolding && !string.IsNullOrEmpty(currentCommand))
        {
            SendCommand(currentCommand);
        }

        //  Manual digital twin movement
        if (manualControl && isHolding)
        {
            float delta = manualSpeed * Time.deltaTime;

            switch (currentCommand)
            {
                case "J1_POS": targetA1 += delta; break;
                case "J1_NEG": targetA1 -= delta; break;

                case "J2_POS": targetA2 += delta; break;
                case "J2_NEG": targetA2 -= delta; break;

                case "J3_POS": targetA3 += delta; break;
                case "J3_NEG": targetA3 -= delta; break;

                case "J4_POS": targetA4 += delta; break;
                case "J4_NEG": targetA4 -= delta; break;

                case "J5_POS": targetA5 += delta; break;
                case "J5_NEG": targetA5 -= delta; break;
            }
        }

        // Smooth movement
        a1 = Mathf.Lerp(a1, targetA1, Time.deltaTime * moveSpeed);
        a2 = Mathf.Lerp(a2, targetA2, Time.deltaTime * moveSpeed);
        a3 = Mathf.Lerp(a3, targetA3, Time.deltaTime * moveSpeed);
        a4 = Mathf.Lerp(a4, targetA4, Time.deltaTime * moveSpeed);
        a5 = Mathf.Lerp(a5, targetA5, Time.deltaTime * moveSpeed);
        g  = Mathf.Lerp(g, targetG, Time.deltaTime * moveSpeed);

        // 🔥 APPLY CALIBRATION OFFSETS
        joint1.localRotation = j1Start * Quaternion.Euler(0f, 0f, -(a1 + offsetJ1));
        joint2.localRotation = j2Start * Quaternion.Euler(0f,  -(a2 - offsetJ2), 0f);
        joint3.localRotation = j3Start * Quaternion.Euler(0f,  (a3 - offsetJ3), 0f);
        joint4.localRotation = j4Start * Quaternion.Euler(0f,  (a4 + offsetJ4), 0f);
        joint5.localRotation = j5Start * Quaternion.Euler(0f,  (a5 + offsetJ5), 0f);

        gripper.localRotation = gStart * Quaternion.Euler(0f, 0f, g);

        UpdateUIText();
    }

    void UpdateUIText()
    {
        if (baseValue != null) baseValue.text = a1.ToString("F1") + "°";
        if (shoulderValue != null) shoulderValue.text = a2.ToString("F1") + "°";
        if (elbowValue != null) elbowValue.text = a3.ToString("F1") + "°";
        if (pitchValue != null) pitchValue.text = a4.ToString("F1") + "°";
        if (rollValue != null) rollValue.text = a5.ToString("F1") + "°";
    }

    // =========================
    // BUTTON CONTROLS
    // =========================

    public void J1_Pos_Down() { currentCommand = "J1_POS"; isHolding = true; manualControl = true; }
    public void J1_Neg_Down() { currentCommand = "J1_NEG"; isHolding = true; manualControl = true; }
    public void J1_Up() { isHolding = false; manualControl = false; SendCommand("J1_STOP"); }

    public void J2_Pos_Down() { currentCommand = "J2_POS"; isHolding = true; manualControl = true; }
    public void J2_Neg_Down() { currentCommand = "J2_NEG"; isHolding = true; manualControl = true; }
    public void J2_Up() { isHolding = false; manualControl = false; SendCommand("J2_STOP"); }

    public void J3_Pos_Down() { currentCommand = "J3_POS"; isHolding = true; manualControl = true; }
    public void J3_Neg_Down() { currentCommand = "J3_NEG"; isHolding = true; manualControl = true; }
    public void J3_Up() { isHolding = false; manualControl = false; SendCommand("J3_STOP"); }

    public void J4_Pos_Down() { currentCommand = "J4_POS"; isHolding = true; manualControl = true; }
    public void J4_Neg_Down() { currentCommand = "J4_NEG"; isHolding = true; manualControl = true; }
    public void J4_Up() { isHolding = false; manualControl = false; SendCommand("J4_STOP"); }

    public void J5_Pos_Down() { currentCommand = "J5_POS"; isHolding = true; manualControl = true; }
    public void J5_Neg_Down() { currentCommand = "J5_NEG"; isHolding = true; manualControl = true; }
    public void J5_Up() { isHolding = false; manualControl = false; SendCommand("J5_STOP"); }

    public void Grip_Open() => SendCommand("GRIP_OPEN");
    public void Grip_Close() => SendCommand("GRIP_CLOSE");

    public void StopAll()
    {
        isHolding = false;
        manualControl = false;
        SendCommand("STOP_ALL");
    }

    // =========================
    // RESET
    // =========================

    public void ResetRobot()
    {
    targetA1 = 0f;
    targetA2 = offsetJ2;
    targetA3 = offsetJ3;
    targetA4 = offsetJ4;
    targetA5 = offsetJ5;
    targetG  = 0f;

    // Apply immediately
    a1 = targetA1;
    a2 = targetA2;
    a3 = targetA3;
    a4 = targetA4;
    a5 = targetA5;
    g  = targetG;

    UpdateUIText();
    }

    public void GoHome()
    {
    isHolding = false;
    manualControl = false;

    SendCommand("HOME");

    targetA1 = 0f;
    targetA2 = -90f;
    targetA3 = 10f;
    targetA4 = 70f;
    targetA5 = 0f;
    }
}
