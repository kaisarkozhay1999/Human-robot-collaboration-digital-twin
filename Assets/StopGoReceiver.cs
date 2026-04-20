using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;

public class StopGoReceiver : MonoBehaviour
{
    [Header("UDP Settings")]
    public int udpPort = 5005;

    [Header("MQTT Settings")]
    public string mqttBrokerIP = "192.168.0.190"; // <- your Pi's IP
    public int mqttBrokerPort = 1883;
    public string mqttTopic = "robot/control";

    public string currentCommand = "none";

    private UdpClient udpClient;
    private IPEndPoint endPoint;
    private ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();

    private MqttClient mqttClient;

    void Start()
    {
        StartUDP();
        StartMQTT();
    }

    // ── UDP ──────────────────────────────────────────────

    void StartUDP()
    {
        try
        {
            endPoint = new IPEndPoint(IPAddress.Any, udpPort);
            udpClient = new UdpClient(udpPort);
            Debug.Log("UDP receiver started on port " + udpPort);
            udpClient.BeginReceive(ReceiveCallback, null);
        }
        catch (Exception e)
        {
            Debug.LogError("Failed to start UDP receiver: " + e.Message);
        }
    }

    private void ReceiveCallback(IAsyncResult ar)
    {
        try
        {
            byte[] data = udpClient.EndReceive(ar, ref endPoint);
            string msg = Encoding.UTF8.GetString(data).Trim().ToLower();
            messageQueue.Enqueue(msg);
            Debug.Log("Received UDP message: " + msg);
            udpClient.BeginReceive(ReceiveCallback, null);
        }
        catch (ObjectDisposedException)
        {
        }
        catch (Exception e)
        {
            Debug.LogError("UDP receive error: " + e.Message);
            if (udpClient != null)
            {
                try { udpClient.BeginReceive(ReceiveCallback, null); }
                catch { }
            }
        }
    }

    // ── MQTT ─────────────────────────────────────────────

    void StartMQTT()
    {
        try
        {
            mqttClient = new MqttClient(mqttBrokerIP, mqttBrokerPort, false, null, null, MqttSslProtocols.None);
            string clientId = "UnityClient_" + Guid.NewGuid().ToString("N").Substring(0, 8);
            mqttClient.Connect(clientId);

            if (mqttClient.IsConnected)
                Debug.Log("MQTT connected to " + mqttBrokerIP + ":" + mqttBrokerPort);
            else
                Debug.LogError("MQTT failed to connect.");
        }
        catch (Exception e)
        {
            Debug.LogError("MQTT connection error: " + e.Message);
        }
    }

    void PublishMQTT(string message)
    {
        if (mqttClient == null || !mqttClient.IsConnected)
        {
            Debug.LogWarning("MQTT not connected. Attempting reconnect...");
            StartMQTT();
        }

        try
        {
            mqttClient.Publish(
                mqttTopic,
                Encoding.UTF8.GetBytes(message),
                MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE,
                false
            );
            Debug.Log("MQTT published: " + message + " -> " + mqttTopic);
        }
        catch (Exception e)
        {
            Debug.LogError("MQTT publish error: " + e.Message);
        }
    }

    // ── UPDATE ───────────────────────────────────────────

    void Update()
    {
        while (messageQueue.TryDequeue(out string msg))
        {
            currentCommand = msg;

            if (msg == "stop")
            {
                Debug.Log("STOP received");
                PublishMQTT("stop");
            }
            else if (msg == "go")
            {
                Debug.Log("GO received");
                PublishMQTT("go");
            }
        }
    }

    // ── CLEANUP ──────────────────────────────────────────

    void OnApplicationQuit()
    {
        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
        }

        if (mqttClient != null && mqttClient.IsConnected)
        {
            mqttClient.Disconnect();
            mqttClient = null;
        }
    }
}