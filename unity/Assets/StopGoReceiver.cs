using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;

public class StopGoReceiver : MonoBehaviour
{
    [Header("UDP Settings")]
    public int udpPort = 5007;

    [Header("MQTT Settings")]
    public string mqttBrokerIP = "192.168.0.248"; // <- your Pi's IP
    public int mqttBrokerPort = 1883;
    public string mqttTopic = "robot/control";

    public string currentCommand = "none";

    private struct QueuedCommand
    {
        public string message;
        public double receiveUnixMs;
        public int bytes;
    }

    private struct MqttConnectResult
    {
        public double durationMs;
        public bool success;
        public string error;
    }

    private UdpClient udpClient;
    private IPEndPoint endPoint;
    private readonly ConcurrentQueue<QueuedCommand> messageQueue = new ConcurrentQueue<QueuedCommand>();
    private readonly ConcurrentQueue<MqttConnectResult> mqttResults = new ConcurrentQueue<MqttConnectResult>();

    private MqttClient mqttClient;
    private int mqttConnecting;
    private volatile bool quitting;


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
            QueuedCommand command = new QueuedCommand();
            command.message = msg;
            command.receiveUnixMs = RuntimeMetricsRecorder.UnixMs();
            command.bytes = data.Length;
            messageQueue.Enqueue(command);
            udpClient.BeginReceive(ReceiveCallback, null);
        }
        catch (ObjectDisposedException) { }
        catch (NullReferenceException) { }
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
        if (Interlocked.Exchange(ref mqttConnecting, 1) != 0) return;
        ThreadPool.QueueUserWorkItem(_ =>
        {
            long started = System.Diagnostics.Stopwatch.GetTimestamp();
            MqttConnectResult result = new MqttConnectResult();
            try
            {
                MqttClient candidate = new MqttClient(mqttBrokerIP, mqttBrokerPort, false, null, null, MqttSslProtocols.None);
                string clientId = "UnityClient_" + Guid.NewGuid().ToString("N").Substring(0, 8);
                candidate.Connect(clientId);
                result.success = candidate.IsConnected;
                if (result.success && !quitting)
                    mqttClient = candidate;
                else if (candidate.IsConnected)
                    candidate.Disconnect();
            }
            catch (Exception e)
            {
                result.error = e.Message;
            }
            result.durationMs = (System.Diagnostics.Stopwatch.GetTimestamp() - started) * 1000.0 / System.Diagnostics.Stopwatch.Frequency;
            mqttResults.Enqueue(result);
            Interlocked.Exchange(ref mqttConnecting, 0);
        });
    }

    void PublishMQTT(string message, double udpReceiveMs, double processMs, int inputBytes)
    {
        if (mqttClient == null || !mqttClient.IsConnected)
        {
            Debug.LogWarning("MQTT not connected. Reconnecting in background.");
            StartMQTT();
            if (RuntimeMetricsRecorder.Instance != null)
                RuntimeMetricsRecorder.Instance.RecordMqtt("publish_skipped", message, 0.0, udpReceiveMs, processMs, inputBytes, false);
            return;
        }

        long started = System.Diagnostics.Stopwatch.GetTimestamp();
        bool success = false;
        try
        {
            mqttClient.Publish(
                mqttTopic,
                Encoding.UTF8.GetBytes(message),
                MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE,
                false
            );
            success = true;
            Debug.Log("MQTT published: " + message + " -> " + mqttTopic);
        }
        catch (Exception e)
        {
            Debug.LogError("MQTT publish error: " + e.Message);
        }

        double durationMs = (System.Diagnostics.Stopwatch.GetTimestamp() - started) * 1000.0 / System.Diagnostics.Stopwatch.Frequency;
        if (RuntimeMetricsRecorder.Instance != null)
            RuntimeMetricsRecorder.Instance.RecordMqtt("publish", message, durationMs, udpReceiveMs, processMs, inputBytes, success);
    }

    public void PublishExternalSafetyCommand(string message)
    {
        if (string.IsNullOrWhiteSpace(message))
            return;

        string normalized = message.Trim().ToLowerInvariant();
        if (normalized != "stop" && normalized != "go")
        {
            Debug.LogWarning("Ignoring unsupported safety command: " + message);
            return;
        }

        double now = RuntimeMetricsRecorder.UnixMs();
        PublishMQTT(normalized, now, now, 0);
    }

    public void PublishSafetyStop()
    {
        PublishExternalSafetyCommand("stop");
    }

    public void PublishSafetyGo()
    {
        PublishExternalSafetyCommand("go");
    }

    // ── UPDATE ───────────────────────────────────────────

    void Update()
    {
        MqttConnectResult connectResult;
        while (mqttResults.TryDequeue(out connectResult))
        {
            if (connectResult.success)
                Debug.Log("MQTT connected to " + mqttBrokerIP + ":" + mqttBrokerPort);
            else
                Debug.LogWarning("MQTT connection failed: " + connectResult.error);
            if (RuntimeMetricsRecorder.Instance != null)
                RuntimeMetricsRecorder.Instance.RecordMqtt("connect", mqttBrokerIP, connectResult.durationMs, 0.0, RuntimeMetricsRecorder.UnixMs(), 0, connectResult.success);
        }

        QueuedCommand command;
        while (messageQueue.TryDequeue(out command))
        {
            double processMs = RuntimeMetricsRecorder.UnixMs();
            string msg = command.message;
            currentCommand = msg;

            if (msg == "stop")
            {
                Debug.Log("STOP received");
                PublishMQTT("stop", command.receiveUnixMs, processMs, command.bytes);
            }
            else if (msg == "go")
            {
                Debug.Log("GO received");
                PublishMQTT("go", command.receiveUnixMs, processMs, command.bytes);
            }
        }
    }

    // ── CLEANUP ──────────────────────────────────────────

    void OnApplicationQuit()
    {
        quitting = true;
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
