using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using System.Text;
using TMPro;
using Newtonsoft.Json;
using System;

[Serializable]
public class SocketTelemetry
{
    public float power;
    public float voltage;
    public float current;
}

public class SocketTelemetrySubscriber : MonoBehaviour
{
    [Header("MQTT Configuration")]
    public string brokerIP = "192.168.0.190";
    public string topic = "lab/room1/socket1/telemetry";

    [Header("UI Reference (TextMeshPro)")]
    public TMP_Text powerText;
    public TMP_Text voltageText;
    public TMP_Text currentText;

    private MqttClient client;
    private SocketTelemetry lastReceivedData;
    private bool hasNewData = false;

    // Object used to lock data across threads
    private readonly object _lock = new object();

    void Start()
    {
        try
        {
            // 1. Initialize Client
            client = new MqttClient(brokerIP);
            
            // 2. Register Message Callback
            client.MqttMsgPublishReceived += OnMqttMessage;

            // 3. Connect with a Unique ID (prevents conflicts)
            string clientId = "Unity_Socket_" + Guid.NewGuid().ToString().Substring(0, 4);
            client.Connect(clientId);

            // 4. Subscribe to the topic
            client.Subscribe(
                new string[] { topic },
                new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE }
            );

            Debug.Log($"<color=green>MQTT Connected!</color> Subscribed to: {topic}");
        }
        catch (Exception e)
        {
            Debug.LogError($"MQTT Connection Failed: {e.Message}");
        }
    }

    // This runs on a BACKGROUND NETWORK THREAD
    void OnMqttMessage(object sender, MqttMsgPublishEventArgs e)
    {
        string rawJson = Encoding.UTF8.GetString(e.Message);
        
        try 
        {
            // Parse the incoming JSON string
            SocketTelemetry data = JsonConvert.DeserializeObject<SocketTelemetry>(rawJson);

            // Thread-safe update of the data object
            lock (_lock)
            {
                lastReceivedData = data;
                hasNewData = true;
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"JSON Error: {ex.Message} | Raw Data: {rawJson}");
        }
    }

    void Update()
    {
        // This runs on the MAIN THREAD (Safe for UI)
        if (hasNewData)
        {
            lock (_lock)
            {
                UpdateUI(lastReceivedData);
                hasNewData = false;
            }
        }
    }

    void UpdateUI(SocketTelemetry data)
    {
        if (data == null) return;

        // We include the "Prefix: " strings here so they don't disappear in-game
        // :F2 rounds to 2 decimal places, :F1 rounds to 1 decimal place
        if (powerText != null) 
            powerText.text = $"Power: {data.power:F2} W";

        if (currentText != null) 
            currentText.text = $"Current: {data.current:F2} A";

        if (voltageText != null) 
            voltageText.text = $"Voltage: {data.voltage:F1} V";
    }

    void OnDestroy()
    {
        // Clean up connection when the game stops
        if (client != null && client.IsConnected)
        {
            client.Disconnect();
        }
    }
}