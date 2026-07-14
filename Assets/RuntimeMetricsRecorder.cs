using System;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

[Serializable]
public class PythonRuntimeMetrics
{
    public double python_frame_start_unix_ms;
    public double python_send_unix_ms;
    public double camera_frame_age_ms;
    public double camera_decode_skew_ms;
    public double inference_ms;
    public double pipeline_ms;
}

[Serializable]
public class SafetyRuntimePacket
{
    public string type;
    public int frame;
    public bool stop;
    public string source;
    public string unit;
    public double distance;
    public PythonRuntimeMetrics metrics;
}

public sealed class RuntimeMetricsRecorder : MonoBehaviour
{
    private class PendingPose
    {
        public int frame;
        public PythonRuntimeMetrics metrics;
        public double received;
        public double applied;
        public int bytes;
    }

    private struct SafetyEnvelope
    {
        public string json;
        public double received;
        public int bytes;
    }

    public static RuntimeMetricsRecorder Instance { get; private set; }
    public string SessionDirectory { get; private set; }

    [Header("Pose3D safety commands")]
    public bool publishPoseSafetyCommands = true;
    public bool publishPoseSafetyGoCommands = true;
    [Tooltip("0 sends only when Pose3D safety changes state. Values above 0 resend the current state at this interval.")]
    public float minimumPoseSafetyCommandIntervalSeconds;
    public string poseSafetyStopCommand = "stop";
    public string poseSafetyGoCommand = "go";

    private readonly ConcurrentQueue<SafetyEnvelope> safetyQueue = new ConcurrentQueue<SafetyEnvelope>();
    private StreamWriter poseWriter;
    private StreamWriter safetyWriter;
    private StreamWriter mqttWriter;
    private PendingPose pendingPose;
    private UdpClient safetyClient;
    private Thread safetyThread;
    private volatile bool safetyRunning;
    private double lastSafetyReceived;
    private StopGoReceiver cachedStopGoReceiver;
    private bool hasLastPoseSafetyStop;
    private bool lastPoseSafetyStop;
    private float lastPoseSafetyCommandTime = float.NegativeInfinity;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void Bootstrap()
    {
        if (Instance != null) return;
        GameObject go = new GameObject("Runtime Metrics Recorder");
        DontDestroyOnLoad(go);
        go.AddComponent<RuntimeMetricsRecorder>();
    }

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }
        Application.runInBackground = true;
        Instance = this;
        DontDestroyOnLoad(gameObject);
        OpenFiles();
        Application.onBeforeRender += BeforeRender;
        StartSafetyReceiver();
    }

    private void OpenFiles()
    {
        string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
        SessionDirectory = Path.Combine(Application.persistentDataPath, "metrics", "unity_" + stamp);
        Directory.CreateDirectory(SessionDirectory);
        poseWriter = Open("unity_frames.csv", "frame,python_frame_start_unix_ms,python_send_unix_ms,unity_receive_unix_ms,unity_apply_unix_ms,unity_before_render_unix_ms,udp_delay_ms,unity_queue_ms,render_queue_ms,software_e2e_ms,camera_frame_age_ms,camera_decode_skew_ms,inference_ms,python_pipeline_ms,packet_bytes,dropped_before_render,unity_frame_count");
        safetyWriter = Open("unity_safety.csv", "frame,stop,source,unit,distance,python_frame_start_unix_ms,python_send_unix_ms,unity_receive_unix_ms,unity_apply_unix_ms,udp_delay_ms,unity_queue_ms,software_e2e_ms,python_pipeline_ms,packet_bytes,interarrival_ms,unity_frame_count");
        mqttWriter = Open("unity_mqtt.csv", "unix_ms,event,message,call_duration_ms,udp_receive_unix_ms,unity_process_unix_ms,callback_to_update_ms,bytes,success,unity_frame_count");
        Debug.Log("Runtime metrics -> " + SessionDirectory);
    }

    private StreamWriter Open(string filename, string header)
    {
        StreamWriter writer = new StreamWriter(Path.Combine(SessionDirectory, filename), false, new UTF8Encoding(false), 65536);
        writer.WriteLine(header);
        writer.Flush();
        return writer;
    }

    public void RecordPoseReceived(int frame, PythonRuntimeMetrics metrics, double receiveUnixMs, int packetBytes)
    {
        if (pendingPose != null) WritePose(pendingPose, 0.0, true);
        pendingPose = new PendingPose();
        pendingPose.frame = frame;
        pendingPose.metrics = metrics;
        pendingPose.received = receiveUnixMs;
        pendingPose.bytes = packetBytes;
    }

    public void RecordPoseApplied(int frame)
    {
        if (pendingPose != null && pendingPose.frame == frame) pendingPose.applied = UnixMs();
    }

    private void BeforeRender()
    {
        if (pendingPose == null || pendingPose.applied <= 0.0) return;
        WritePose(pendingPose, UnixMs(), false);
        pendingPose = null;
    }

    private void WritePose(PendingPose pose, double beforeRender, bool dropped)
    {
        PythonRuntimeMetrics m = pose.metrics;
        double start = m == null ? 0.0 : m.python_frame_start_unix_ms;
        double sent = m == null ? 0.0 : m.python_send_unix_ms;
        string[] row = {
            I(pose.frame), D(start), D(sent), D(pose.received), D(pose.applied), D(beforeRender),
            Delta(pose.received, sent), Delta(pose.applied, pose.received), Delta(beforeRender, pose.applied),
            Delta(beforeRender, start), D(m == null ? 0.0 : m.camera_frame_age_ms),
            D(m == null ? 0.0 : m.camera_decode_skew_ms), D(m == null ? 0.0 : m.inference_ms),
            D(m == null ? 0.0 : m.pipeline_ms), I(pose.bytes), dropped ? "1" : "0", I(Time.frameCount)
        };
        poseWriter.WriteLine(string.Join(",", row));
    }

    private void StartSafetyReceiver()
    {
        try
        {
            safetyClient = new UdpClient(5006);
            safetyRunning = true;
            safetyThread = new Thread(SafetyLoop);
            safetyThread.IsBackground = true;
            safetyThread.Start();
            Debug.Log("Runtime safety metrics listening on UDP :5006");
        }
        catch (Exception ex)
        {
            Debug.LogWarning("Safety metrics UDP bind failed: " + ex.Message);
        }
    }

    private void SafetyLoop()
    {
        IPEndPoint endpoint = new IPEndPoint(IPAddress.Any, 0);
        while (safetyRunning)
        {
            try
            {
                byte[] data = safetyClient.Receive(ref endpoint);
                SafetyEnvelope item = new SafetyEnvelope();
                item.json = Encoding.UTF8.GetString(data);
                item.received = UnixMs();
                item.bytes = data.Length;
                safetyQueue.Enqueue(item);
            }
            catch (ObjectDisposedException) { break; }
            catch (SocketException) { if (!safetyRunning) break; }
            catch (Exception ex) { if (safetyRunning) Debug.LogWarning("Safety metrics UDP: " + ex.Message); }
        }
    }

    private void Update()
    {
        SafetyEnvelope item;
        while (safetyQueue.TryDequeue(out item))
        {
            double applied = UnixMs();
            SafetyRuntimePacket packet;
            try { packet = JsonUtility.FromJson<SafetyRuntimePacket>(item.json); }
            catch { continue; }
            if (packet == null || packet.type != "robot_safety") continue;
            PythonRuntimeMetrics m = packet.metrics;
            double start = m == null ? 0.0 : m.python_frame_start_unix_ms;
            double sent = m == null ? 0.0 : m.python_send_unix_ms;
            double interval = lastSafetyReceived > 0.0 ? item.received - lastSafetyReceived : 0.0;
            lastSafetyReceived = item.received;
            string[] row = {
                I(packet.frame), packet.stop ? "1" : "0", Clean(packet.source), Clean(packet.unit), D(packet.distance),
                D(start), D(sent), D(item.received), D(applied), Delta(item.received, sent),
                Delta(applied, item.received), Delta(applied, start), D(m == null ? 0.0 : m.pipeline_ms),
                I(item.bytes), D(interval), I(Time.frameCount)
            };
            safetyWriter.WriteLine(string.Join(",", row));
            ApplyPoseSafetyCommand(packet, item.received, applied, item.bytes);
        }
    }

    private void ApplyPoseSafetyCommand(SafetyRuntimePacket packet, double receiveMs, double processMs, int packetBytes)
    {
        if (!publishPoseSafetyCommands || packet == null)
            return;

        if (!packet.stop && !publishPoseSafetyGoCommands)
        {
            hasLastPoseSafetyStop = true;
            lastPoseSafetyStop = packet.stop;
            return;
        }

        bool stateChanged = !hasLastPoseSafetyStop || packet.stop != lastPoseSafetyStop;
        float minInterval = Mathf.Max(0f, minimumPoseSafetyCommandIntervalSeconds);
        bool intervalReady = minInterval > 0f && Time.time - lastPoseSafetyCommandTime >= minInterval;

        if (!stateChanged && !intervalReady)
            return;

        string command = packet.stop ? poseSafetyStopCommand : poseSafetyGoCommand;
        PublishPoseSafetyCommand(command, packet.stop, stateChanged, receiveMs, processMs, packetBytes);

        hasLastPoseSafetyStop = true;
        lastPoseSafetyStop = packet.stop;
        lastPoseSafetyCommandTime = Time.time;
    }

    private void PublishPoseSafetyCommand(string command, bool stop, bool stateChanged, double receiveMs, double processMs, int packetBytes)
    {
        if (string.IsNullOrWhiteSpace(command))
            return;

        bool sent = false;
        StopGoReceiver receiver = FindStopGoReceiver();
        if (receiver != null)
        {
            sent = receiver.PublishExternalSafetyCommand(command);
        }

        ER9ProFullController[] controllers = FindObjectsOfType<ER9ProFullController>();
        if (stop)
        {
            foreach (ER9ProFullController controller in controllers)
            {
                if (controller == null)
                    continue;

                controller.StopAll();
                sent = true;
            }
        }
        else if (receiver == null)
        {
            foreach (ER9ProFullController controller in controllers)
            {
                if (controller == null)
                    continue;

                controller.PublishControlCommand(command);
                sent = true;
            }
        }

        if (stateChanged || !sent)
        {
            string state = stop ? "STOP" : "GO";
            string targetStatus = sent ? "sent" : "no Unity MQTT target found";
            Debug.Log("Pose3D safety " + state + " " + targetStatus + " from UDP :5006");
        }
    }

    private StopGoReceiver FindStopGoReceiver()
    {
        if (cachedStopGoReceiver != null)
            return cachedStopGoReceiver;

        cachedStopGoReceiver = FindObjectOfType<StopGoReceiver>();
        return cachedStopGoReceiver;
    }

    public void RecordMqtt(string eventName, string message, double durationMs, double udpReceiveMs, double processMs, int bytes, bool success)
    {
        if (mqttWriter == null) return;
        string[] row = {
            D(UnixMs()), Clean(eventName), Clean(message), D(durationMs), D(udpReceiveMs), D(processMs),
            Delta(processMs, udpReceiveMs), I(bytes), success ? "1" : "0", I(Time.frameCount)
        };
        mqttWriter.WriteLine(string.Join(",", row));
    }

    public static double UnixMs()
    {
        return (DateTime.UtcNow.Ticks - 621355968000000000L) / 10000.0;
    }

    private static string D(double value)
    {
        return value > 0.0 ? value.ToString("0.####", CultureInfo.InvariantCulture) : "";
    }

    private static string I(int value)
    {
        return value.ToString(CultureInfo.InvariantCulture);
    }

    private static string Delta(double later, double earlier)
    {
        if (later <= 0.0 || earlier <= 0.0) return "";
        return (later - earlier).ToString("0.####", CultureInfo.InvariantCulture);
    }


    private static string Clean(string value)
    {
        return string.IsNullOrEmpty(value) ? "" : value.Replace(",", ";");
    }

    private void Shutdown()
    {
        Application.onBeforeRender -= BeforeRender;
        safetyRunning = false;
        if (safetyClient != null) safetyClient.Close();
        safetyClient = null;
        if (safetyThread != null && safetyThread.IsAlive) safetyThread.Join(300);
        safetyThread = null;
        if (pendingPose != null) WritePose(pendingPose, 0.0, true);
        pendingPose = null;
        StreamWriter[] writers = { poseWriter, safetyWriter, mqttWriter };
        foreach (StreamWriter writer in writers)
        {
            if (writer == null) continue;
            writer.Flush();
            writer.Dispose();
        }
        poseWriter = null;
        safetyWriter = null;
        mqttWriter = null;
    }

    private void OnDisable()
    {
        Shutdown();
        if (Instance == this) Instance = null;
    }

    private void OnApplicationPause(bool paused)
    {
        if (!paused) return;
        if (poseWriter != null) poseWriter.Flush();
        if (safetyWriter != null) safetyWriter.Flush();
        if (mqttWriter != null) mqttWriter.Flush();
    }
}
