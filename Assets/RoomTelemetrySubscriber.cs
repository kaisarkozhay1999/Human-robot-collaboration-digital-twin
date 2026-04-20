using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using System.Text;
using TMPro;
using Newtonsoft.Json;
using System;

[Serializable]
public class RoomTelemetry
{
    public float temperature;
    public float humidity;
    public float pressure;
    public float light;
    public string motion;
}

public class RoomTelemetrySubscriber : MonoBehaviour
{
    [Header("MQTT Configuration")]
    public string brokerIP = "192.168.0.190";
    public string topic = "lab/sensors/all";

    [Header("UI References (TextMeshPro)")]
    public TMP_Text temperatureText;
    public TMP_Text humidityText;
    public TMP_Text pressureText;
    public TMP_Text lightText;
    public TMP_Text motionText;

    private MqttClient client;
    private RoomTelemetry lastReceivedData;
    private bool hasNewData = false;

    private readonly object _lock = new object();

    void Start()
    {
        try
        {
            // Create client
            client = new MqttClient(brokerIP);

            // Register callback
            client.MqttMsgPublishReceived += OnMqttMessage;

            // Unique client ID (important)
            string clientId = "Unity_Room_" + Guid.NewGuid().ToString().Substring(0, 4);
            client.Connect(clientId);

            // Subscribe
            client.Subscribe(
                new string[] { topic },
                new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE }
            );

            Debug.Log($"<color=green>Room MQTT Connected!</color> Subscribed to: {topic}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Room MQTT Connection Failed: {e.Message}");
        }
    }

    // Background network thread
    void OnMqttMessage(object sender, MqttMsgPublishEventArgs e)
    {
        string rawJson = Encoding.UTF8.GetString(e.Message);

        try
        {
            RoomTelemetry data = JsonConvert.DeserializeObject<RoomTelemetry>(rawJson);

            lock (_lock)
            {
                lastReceivedData = data;
                hasNewData = true;
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"Room JSON Error: {ex.Message} | Raw Data: {rawJson}");
        }
    }

    void Update()
    {
        if (hasNewData)
        {
            lock (_lock)
            {
                UpdateUI(lastReceivedData);
                hasNewData = false;
            }
        }
    }

    void UpdateUI(RoomTelemetry data)
    {
        if (data == null) return;

        if (temperatureText != null)
            temperatureText.text = $"Temperature: {data.temperature:F1} °C";

        if (humidityText != null)
            humidityText.text = $"Humidity: {data.humidity:F1} %";

        if (pressureText != null)
            pressureText.text = $"Pressure: {data.pressure:F2} psi";

        if (lightText != null)
            lightText.text = $"Light: {data.light:F1} lx";

        if (motionText != null)
            motionText.text = $"Motion: {(data.motion == "on" ? "Detected" : "Clear")}";
    }

    void OnDestroy()
    {
        if (client != null && client.IsConnected)
        {
            client.Disconnect();
        }
    }
}
