using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace DigitalTwin.Kinematics
{
    [DefaultExecutionOrder(200)]
    public sealed class RobotKinematicsCsvLogger : MonoBehaviour
    {
        public RobotSafetyDistanceEvaluator evaluator;
        public bool logOnStart = true;
        public bool logEveryFrame;
        public float sampleIntervalSeconds = 0.1f;
        public int flushEveryRows = 30;

        [Tooltip("Optional Unity visual TCP marker. If assigned, the CSV includes FK-vs-Unity TCP error.")]
        public Transform unityTcpMarker;

        public string SessionDirectory { get; private set; }
        public string CsvPath { get; private set; }

        private StreamWriter writer;
        private float nextSampleTime;
        private int rowsSinceFlush;
        private bool isLogging;

        private void Awake()
        {
            if (evaluator == null)
                evaluator = GetComponent<RobotSafetyDistanceEvaluator>();
        }

        private void Start()
        {
            if (logOnStart)
                StartLogging();
        }

        private void Update()
        {
            if (!isLogging || evaluator == null)
                return;

            if (!logEveryFrame && Time.time < nextSampleTime)
                return;

            nextSampleTime = Time.time + Mathf.Max(0.001f, sampleIntervalSeconds);
            WriteRow();
        }

        [ContextMenu("Start FK CSV Logging")]
        public void StartLogging()
        {
            if (isLogging)
                return;

            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            SessionDirectory = Path.Combine(Application.persistentDataPath, "metrics", "fk_safety_" + stamp);
            Directory.CreateDirectory(SessionDirectory);
            CsvPath = Path.Combine(SessionDirectory, "fk_safety.csv");
            writer = new StreamWriter(CsvPath, false, new UTF8Encoding(false), 65536);
            WriteHeader();
            isLogging = true;
            nextSampleTime = Time.time;
            Debug.Log("FK safety CSV -> " + CsvPath);
        }

        [ContextMenu("Stop FK CSV Logging")]
        public void StopLogging()
        {
            if (!isLogging && writer == null)
                return;

            isLogging = false;
            writer?.Flush();
            writer?.Dispose();
            writer = null;
        }

        private void WriteHeader()
        {
            writer.WriteLine(string.Join(",", new[]
            {
                "unity_frame",
                "time_seconds",
                "unix_ms",
                "decision",
                "robot_data_fresh",
                "human_data_fresh",
                "data_fresh",
                "min_distance_m",
                "closest_human_joint_index",
                "closest_robot_link_index",
                "closest_human_x",
                "closest_human_y",
                "closest_human_z",
                "closest_robot_x",
                "closest_robot_y",
                "closest_robot_z",
                "fk_tcp_x",
                "fk_tcp_y",
                "fk_tcp_z",
                "unity_tcp_x",
                "unity_tcp_y",
                "unity_tcp_z",
                "fk_vs_unity_tcp_error_m",
                "joint_angles_deg",
                "fk_joint_positions",
                "fk_link_segments",
                "human_joint_positions",
                "gesture_stop_requested",
                "gesture_go_requested",
                "gesture_command",
                "gesture_command_age_sec"
            }));
        }

        private void WriteRow()
        {
            if (writer == null)
                return;

            ForwardKinematicsResult fk = TryBuildModelKinematics(out ForwardKinematicsResult modelKinematics)
                ? modelKinematics
                : evaluator.CurrentKinematics;
            HumanRobotDistanceResult distance = evaluator.CurrentDistance;

            Vector3 tcp = fk.JointPositions == null ? NanVector() : fk.EndEffectorPosition;
            Vector3 unityTcp = unityTcpMarker == null ? NanVector() : unityTcpMarker.position;
            float tcpError = unityTcpMarker == null || fk.JointPositions == null
                ? float.NaN
                : Vector3.Distance(tcp, unityTcp);

            string[] row =
            {
                I(Time.frameCount),
                F(Time.time),
                D(UnixMs()),
                evaluator.CurrentDecision.ToString(),
                B(evaluator.RobotDataFresh),
                B(evaluator.HumanDataFresh),
                B(evaluator.DataFresh),
                F(distance.IsValid ? distance.DistanceMeters : float.NaN),
                I(distance.IsValid ? distance.HumanJointIndex : -1),
                I(distance.IsValid ? distance.RobotLinkIndex : -1),
                F(distance.IsValid ? distance.HumanJointPosition.x : float.NaN),
                F(distance.IsValid ? distance.HumanJointPosition.y : float.NaN),
                F(distance.IsValid ? distance.HumanJointPosition.z : float.NaN),
                F(distance.IsValid ? distance.ClosestPointOnRobotLink.x : float.NaN),
                F(distance.IsValid ? distance.ClosestPointOnRobotLink.y : float.NaN),
                F(distance.IsValid ? distance.ClosestPointOnRobotLink.z : float.NaN),
                F(tcp.x),
                F(tcp.y),
                F(tcp.z),
                F(unityTcp.x),
                F(unityTcp.y),
                F(unityTcp.z),
                F(tcpError),
                Csv(evaluator.latestJointAnglesDegrees),
                Csv(fk.JointPositions),
                Csv(fk.LinkSegments),
                Csv(evaluator.CurrentHumanJointPositions),
                B(evaluator.gestureStopRequested),
                B(evaluator.gestureGoRequested),
                Quote(evaluator.gestureReceiver == null ? "" : evaluator.gestureReceiver.currentCommand),
                F(evaluator.gestureReceiver == null ? float.NaN : evaluator.gestureReceiver.CommandAgeSeconds)
            };

            writer.WriteLine(string.Join(",", row));
            rowsSinceFlush++;
            if (rowsSinceFlush >= Mathf.Max(1, flushEveryRows))
            {
                rowsSinceFlush = 0;
                writer.Flush();
            }
        }

        private bool TryBuildModelKinematics(out ForwardKinematicsResult kinematics)
        {
            kinematics = default;
            if (evaluator == null || evaluator.parameters == null || evaluator.latestJointAnglesDegrees == null)
                return false;

            if (evaluator.useRobotControllerPositionAsBase && evaluator.robotController != null)
            {
                evaluator.parameters.basePositionMeters =
                    evaluator.robotController.transform.position + evaluator.kinematicBaseOffsetMeters;
            }

            kinematics = RobotKinematics.ForwardKinematics(
                evaluator.parameters,
                evaluator.latestJointAnglesDegrees);

            return kinematics.JointPositions != null && kinematics.JointPositions.Length > 0;
        }

        private static Vector3 NanVector()
        {
            return new Vector3(float.NaN, float.NaN, float.NaN);
        }

        private static string Csv(float[] values)
        {
            if (values == null || values.Length == 0)
                return "\"\"";

            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < values.Length; i++)
            {
                if (i > 0) builder.Append('|');
                builder.Append(F(values[i]));
            }

            return Quote(builder.ToString());
        }

        private static string Csv(Vector3[] values)
        {
            if (values == null || values.Length == 0)
                return "\"\"";

            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < values.Length; i++)
            {
                if (i > 0) builder.Append('|');
                AppendVector(builder, values[i]);
            }

            return Quote(builder.ToString());
        }

        private static string Csv(RobotLinkSegment[] segments)
        {
            if (segments == null || segments.Length == 0)
                return "\"\"";

            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < segments.Length; i++)
            {
                if (i > 0) builder.Append('|');
                builder.Append(segments[i].Name.Replace(",", ";"));
                builder.Append(':');
                AppendVector(builder, segments[i].Start);
                builder.Append('>');
                AppendVector(builder, segments[i].End);
            }

            return Quote(builder.ToString());
        }

        private static void AppendVector(StringBuilder builder, Vector3 value)
        {
            builder.Append(F(value.x));
            builder.Append(' ');
            builder.Append(F(value.y));
            builder.Append(' ');
            builder.Append(F(value.z));
        }

        private static string Quote(string value)
        {
            return "\"" + value.Replace("\"", "\"\"") + "\"";
        }

        private static string B(bool value)
        {
            return value ? "1" : "0";
        }

        private static string I(int value)
        {
            return value.ToString(CultureInfo.InvariantCulture);
        }

        private static string D(double value)
        {
            return double.IsNaN(value) || double.IsInfinity(value)
                ? ""
                : value.ToString("0.####", CultureInfo.InvariantCulture);
        }

        private static string F(float value)
        {
            return float.IsNaN(value) || float.IsInfinity(value)
                ? ""
                : value.ToString("0.######", CultureInfo.InvariantCulture);
        }

        private static double UnixMs()
        {
            return (DateTime.UtcNow.Ticks - 621355968000000000L) / 10000.0;
        }

        private void OnDisable()
        {
            StopLogging();
        }

        private void OnApplicationQuit()
        {
            StopLogging();
        }
    }
}
