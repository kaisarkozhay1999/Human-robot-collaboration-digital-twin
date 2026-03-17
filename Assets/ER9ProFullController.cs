using UnityEngine;
using MixedReality.Toolkit.UX;
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

    [Header("UI Text (Degree Display)")]
    public TMP_Text baseValue;
    public TMP_Text shoulderValue;
    public TMP_Text elbowValue;
    public TMP_Text pitchValue;
    public TMP_Text rollValue;

    [Header("MRTK Sliders")]
    public Slider baseSlider;
    public Slider shoulderSlider;
    public Slider elbowSlider;
    public Slider pitchSlider;
    public Slider rollSlider;

    [Header("Movement Settings")]
    public float moveSpeed = 5f;

    [Header("MQTT Settings")]
    public string brokerIP = "192.168.0.190";   // CHANGE to your Raspberry Pi or broker IP
    public string topic = "robot/state";

    MqttClient client;

    float a1, a2, a3, a4, a5, g;
    float targetA1, targetA2, targetA3, targetA4, targetA5, targetG;

    Quaternion j1Start, j2Start, j3Start, j4Start, j5Start, gStart;

    void Start()
    {
        j1Start = joint1.localRotation;
        j2Start = joint2.localRotation;
        j3Start = joint3.localRotation;
        j4Start = joint4.localRotation;
        j5Start = joint5.localRotation;
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

            client.Subscribe(new string[] { topic }, new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE });

            Debug.Log("MQTT Connected");
        }
        catch (System.Exception e)
        {
            Debug.Log("MQTT ERROR: " + e.Message);
        }
    }

    void OnMessageReceived(object sender, MqttMsgPublishEventArgs e)
    {
        string msg = Encoding.UTF8.GetString(e.Message);

        Debug.Log("MQTT Received: " + msg);

        // Expected format:
        // 10,20,30,40,50,0

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
        a1 = Mathf.Lerp(a1, targetA1, Time.deltaTime * moveSpeed);
        a2 = Mathf.Lerp(a2, targetA2, Time.deltaTime * moveSpeed);
        a3 = Mathf.Lerp(a3, targetA3, Time.deltaTime * moveSpeed);
        a4 = Mathf.Lerp(a4, targetA4, Time.deltaTime * moveSpeed);
        a5 = Mathf.Lerp(a5, targetA5, Time.deltaTime * moveSpeed);
        g  = Mathf.Lerp(g, targetG, Time.deltaTime * moveSpeed);

        joint1.localRotation = j1Start * Quaternion.Euler(0f, 0f, -a1);
        joint2.localRotation = j2Start * Quaternion.Euler(0f, a2, 0f);
        joint3.localRotation = j3Start * Quaternion.Euler(0f, a3, 0f);
        joint4.localRotation = j4Start * Quaternion.Euler(0f, a4, 0f);
        joint5.localRotation = j5Start * Quaternion.Euler(0f, a5, 0f);

        gripper.localRotation = gStart * Quaternion.Euler(0f, 0f, g);
    }

    // =========================
    // SLIDER CONTROL FUNCTIONS
    // =========================

    public void SetJoint1(SliderEventData data)
    {
        targetA1 = Mathf.Lerp(-138f, 138f, data.NewValue);
        if (baseValue != null)
            baseValue.text = targetA1.ToString("F1") + "°";
    }

    public void SetJoint2(SliderEventData data)
    {
        targetA2 = Mathf.Lerp(-76.5f, 76.5f, data.NewValue);
        if (shoulderValue != null)
            shoulderValue.text = targetA2.ToString("F1") + "°";
    }

    public void SetJoint3(SliderEventData data)
    {
        targetA3 = Mathf.Lerp(-107f, 107f, data.NewValue);
        if (elbowValue != null)
            elbowValue.text = targetA3.ToString("F1") + "°";
    }

    public void SetJoint4(SliderEventData data)
    {
        targetA4 = Mathf.Lerp(-101f, 101f, data.NewValue);
        if (pitchValue != null)
            pitchValue.text = targetA4.ToString("F1") + "°";
    }

    public void SetJoint5(SliderEventData data)
    {
        targetA5 = Mathf.Lerp(-368f, 368f, data.NewValue);
        if (rollValue != null)
            rollValue.text = targetA5.ToString("F1") + "°";
    }

    public void SetGripper(SliderEventData data)
    {
        targetG = Mathf.Lerp(-45f, 45f, data.NewValue);
    }

    // =========================
    // RESET FUNCTION
    // =========================

    public void ResetRobot()
    {
        targetA1 = 0;
        targetA2 = 0;
        targetA3 = 0;
        targetA4 = 0;
        targetA5 = 0;
        targetG  = 0;

        if (baseValue != null) baseValue.text = "0°";
        if (shoulderValue != null) shoulderValue.text = "0°";
        if (elbowValue != null) elbowValue.text = "0°";
        if (pitchValue != null) pitchValue.text = "0°";
        if (rollValue != null) rollValue.text = "0°";

        if (baseSlider != null) baseSlider.Value = 0.5f;
        if (shoulderSlider != null) shoulderSlider.Value = 0.5f;
        if (elbowSlider != null) elbowSlider.Value = 0.5f;
        if (pitchSlider != null) pitchSlider.Value = 0.5f;
        if (rollSlider != null) rollSlider.Value = 0.5f;
    }
}