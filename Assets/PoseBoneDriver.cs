using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.SceneManagement;

[Serializable]
public class PoseBonePacket2
{
    public string type;
    public int frame;
    public string coordinate_space;
    public ArucoMarkerPointBone2[] aruco_markers;
    public CameraPointBone2[] cameras;
    public PersonPoseBone2[] people;
    public JointPointBone2[] joints;
    public PythonRuntimeMetrics metrics;
}

[Serializable]
public class JointPointBone2
{
    public int id;
    public string name;
    public bool tracked;
    public float confidence;
    public float x, y, z;
}

[Serializable]
public class ArucoMarkerPointBone2
{
    public int id;
    public bool tracked;
    public float confidence;
    public float x, y, z;
}

[Serializable]
public class CameraPointBone2
{
    public int id;
    public string name;
    public bool tracked;
    public float x, y, z;
    public bool hasOrientation;
    public float rightX, rightY, rightZ;
    public float upX, upY, upZ;
    public float forwardX, forwardY, forwardZ;
}

[Serializable]
public class PersonPoseBone2
{
    public int id;
    public bool tracked;
    public float confidence;
    public JointPointBone2[] joints;
}

public class PoseBoneDriver : MonoBehaviour
{
    const int NOSE = 0;
    const int LEFT_SHOULDER = 5;
    const int RIGHT_SHOULDER = 6;
    const int LEFT_ELBOW = 7;
    const int RIGHT_ELBOW = 8;
    const int LEFT_WRIST = 9;
    const int RIGHT_WRIST = 10;
    const int LEFT_HIP = 11;
    const int RIGHT_HIP = 12;
    const int LEFT_KNEE = 13;
    const int RIGHT_KNEE = 14;
    const int LEFT_ANKLE = 15;
    const int RIGHT_ANKLE = 16;

    [Header("UDP")]
    public int listenPort = 5005;
    public bool receiveUdp = true;

    [Header("Multi Person Avatars")]
    public bool spawnAvatarsForPeople = true;
    public float avatarDisappearSeconds = 0.75f;

    [Header("Skeleton Overlay")]
    public bool showSkeleton = true;
    public float jointSize = 0.02f;
    public float boneWidth = 0.005f;
    public Color jointColor = Color.cyan;
    public Color boneColor = Color.green;

    [Header("Coordinate Tuning")]
    public float poseScale = 0.1f;
    public Vector3 poseOffset = Vector3.zero;
    public Vector3 rotationOffsetEuler = Vector3.zero;
    public bool swapXZ = true;
    public bool mirrorX = false;
    public bool invertY = false;
    public bool invertZ = false;

    [Header("ArUco Anchor Mapping")]
    public bool useArucoAnchors = true;
    public bool autoFindArucoAnchors = true;
    public string arucoMarkerNamePrefix = "ArucoMarker_";
    public Transform[] arucoAnchors = new Transform[50];
    public float singleAnchorScale = 0.1f;
    public float minAnchorScale = 0.005f;
    public float maxAnchorScale = 2.0f;
    public Vector3 anchorMappedOffset = Vector3.zero;
    public bool updateMappedCameraObjects = false;
    public Transform mappedCamera1;
    public Transform mappedCamera2;

    [Header("Smoothing (0 = raw)")]
    public float smoothing = 10f;

    [Header("Grounding")]
    public bool groundToAnkles = true;
    public float floorY = 0f;
    public float maxGroundCorrection = 2f;
    public float minHipHeightAboveFloor = 0.1f;

    [Header("Root Motion")]
    public bool driveRootXZ = true;
    public bool lockRootRotation = false;
    public bool driveRootYaw = true;
    public float rootYawOffsetDegrees = 0f;
    public float maxRootSpeed = 0.75f;
    public float maxRootSnapDistance = 0.4f;

    [Header("Body Rotation")]
    public bool driveHipsRotation = true;
    public bool driveSpineRotation = false;

    [Header("Leg Bend Correction")]
    public bool flipLeftLegBend = false;
    public bool flipRightLegBend = false;

    [Header("Depth Stabilization")]
    public bool stabilizeArmDepth = false;
    [Range(0f, 1f)] public float armDepthWeight = 0.1f;
    public bool stabilizeLegDepth = false;
    [Range(0f, 1f)] public float legDepthWeight = 0.25f;

    [Header("Confidence")]
    public float minConfidence = 0.25f;

    // ── Bones ─────────────────────────────────────────────────
    private Transform boneHips;
    private Transform boneSpine, boneSpine1, boneSpine2;
    private Transform boneNeck, boneHead;
    private Transform boneLeftArm, boneLeftForeArm;
    private Transform boneRightArm, boneRightForeArm;
    private Transform boneLeftHand, boneRightHand;
    private Transform boneLeftUpLeg, boneLeftLeg;
    private Transform boneRightUpLeg, boneRightLeg;
    private Transform boneLeftFoot, boneRightFoot;

    // Bind pose local rotations
    private Quaternion bindHips;
    private Quaternion bindSpine, bindSpine1, bindSpine2;
    private Quaternion bindLeftArm, bindLeftForeArm;
    private Quaternion bindRightArm, bindRightForeArm;
    private Quaternion bindLeftUpLeg, bindLeftLeg;
    private Quaternion bindRightUpLeg, bindRightLeg;

    // Bind pose bone axes. Mixamo limbs usually do not point along Unity +Z.
    private Vector3 bindSpineAxis = Vector3.up;
    private Vector3 bindSpine1Axis = Vector3.up;
    private Vector3 bindSpine2Axis = Vector3.up;
    private Vector3 bindLeftArmAxis = Vector3.right;
    private Vector3 bindLeftForeArmAxis = Vector3.right;
    private Vector3 bindRightArmAxis = Vector3.left;
    private Vector3 bindRightForeArmAxis = Vector3.left;
    private Vector3 bindLeftUpLegAxis = Vector3.down;
    private Vector3 bindLeftLegAxis = Vector3.down;
    private Vector3 bindRightUpLegAxis = Vector3.down;
    private Vector3 bindRightLegAxis = Vector3.down;

    // Smoothed current local rotations
    private Quaternion curHips = Quaternion.identity;
    private Quaternion curSpine = Quaternion.identity;
    private Quaternion curSpine1 = Quaternion.identity;
    private Quaternion curSpine2 = Quaternion.identity;
    private Quaternion curLeftArm = Quaternion.identity;
    private Quaternion curLeftForeArm = Quaternion.identity;
    private Quaternion curRightArm = Quaternion.identity;
    private Quaternion curRightForeArm = Quaternion.identity;
    private Quaternion curLeftUpLeg = Quaternion.identity;
    private Quaternion curLeftLeg = Quaternion.identity;
    private Quaternion curRightUpLeg = Quaternion.identity;
    private Quaternion curRightLeg = Quaternion.identity;

    // Joint data
    private Vector3[] joints = new Vector3[17];
    private bool[] active = new bool[17];
    private Vector3 groundOffset = Vector3.zero;
    private Transform motionRoot;
    private Vector3 bindRootToHipOffset;
    private bool rootPositionInitialized;

    // UDP
    private UdpClient udpClient;
    private Thread receiveThread;
    private volatile bool running;
    private readonly object jsonLock = new object();
    private string latestJson = null;
    private double latestReceiveUnixMs;
    private int latestPacketBytes;
    private int lastFrameId = -1;
    private int assignedPersonId = -1;
    private bool isSpawnedAvatar = false;
    private readonly Dictionary<int, PoseBoneDriver> avatarDrivers = new Dictionary<int, PoseBoneDriver>();
    private readonly Dictionary<int, float> avatarLastSeen = new Dictionary<int, float>();

    // Live similarity transform from incoming real coordinates to adjusted Unity anchors.
    private bool anchorMappingValid;
    private Vector3 anchorRealCentroid;
    private Vector3 anchorUnityCentroid;
    private Quaternion anchorRotation = Quaternion.identity;
    private float anchorScale = 1f;
    private bool packetCoordinatesAreUnityWorld;

    // Overlay
    private GameObject[] debugSpheres;
    private LineRenderer[] debugLines;
    private float lastJointSize = -1f;

    public int JointCount => joints == null ? 0 : joints.Length;
    public int LastAppliedFrameId => lastFrameId;
    public float LastPoseUpdateTime { get; private set; } = float.NegativeInfinity;

    private readonly int[,] skeletonDef = {
        {LEFT_SHOULDER,  RIGHT_SHOULDER},
        {LEFT_SHOULDER,  LEFT_ELBOW},
        {LEFT_ELBOW,     LEFT_WRIST},
        {RIGHT_SHOULDER, RIGHT_ELBOW},
        {RIGHT_ELBOW,    RIGHT_WRIST},
        {LEFT_SHOULDER,  LEFT_HIP},
        {RIGHT_SHOULDER, RIGHT_HIP},
        {LEFT_HIP,       RIGHT_HIP},
        {LEFT_HIP,       LEFT_KNEE},
        {LEFT_KNEE,      LEFT_ANKLE},
        {RIGHT_HIP,      RIGHT_KNEE},
        {RIGHT_KNEE,     RIGHT_ANKLE},
    };

    private void Start()
    {
        AutoFindAnchorObjects();
        FindBones();
        CaptureBindPose();
        CaptureRootMotionReference();
        CreateOverlay();
        if (receiveUdp)
            StartReceiver();
    }

    private void OnDisable() => StopReceiver();
    private void OnDestroy() => StopReceiver();
    private void OnApplicationQuit() => StopReceiver();

    // ── Find bones ────────────────────────────────────────────
    private Transform FindBone(string n)
    {
        foreach (Transform t in GetComponentsInChildren<Transform>(true))
            if (t.name == n) return t;
        Debug.LogWarning($"Bone not found: {n}");
        return null;
    }

    private void FindBones()
    {
        boneHips = FindBone("mixamorig:Hips");
        boneSpine = FindBone("mixamorig:Spine");
        boneSpine1 = FindBone("mixamorig:Spine1");
        boneSpine2 = FindBone("mixamorig:Spine2");
        boneNeck = FindBone("mixamorig:Neck");
        boneHead = FindBone("mixamorig:Head");
        boneLeftArm = FindBone("mixamorig:LeftArm");
        boneLeftForeArm = FindBone("mixamorig:LeftForeArm");
        boneRightArm = FindBone("mixamorig:RightArm");
        boneRightForeArm = FindBone("mixamorig:RightForeArm");
        boneLeftHand = FindBone("mixamorig:LeftHand");
        boneRightHand = FindBone("mixamorig:RightHand");
        boneLeftUpLeg = FindBone("mixamorig:LeftUpLeg");
        boneLeftLeg = FindBone("mixamorig:LeftLeg");
        boneRightUpLeg = FindBone("mixamorig:RightUpLeg");
        boneRightLeg = FindBone("mixamorig:RightLeg");
        boneLeftFoot = FindBone("mixamorig:LeftFoot");
        boneRightFoot = FindBone("mixamorig:RightFoot");
    }

    // ── Capture bind pose ─────────────────────────────────────
    private void CaptureBindPose()
    {
        bindHips = Safe(boneHips);
        bindSpine = Safe(boneSpine);
        bindSpine1 = Safe(boneSpine1);
        bindSpine2 = Safe(boneSpine2);
        bindLeftArm = Safe(boneLeftArm);
        bindLeftForeArm = Safe(boneLeftForeArm);
        bindRightArm = Safe(boneRightArm);
        bindRightForeArm = Safe(boneRightForeArm);
        bindLeftUpLeg = Safe(boneLeftUpLeg);
        bindLeftLeg = Safe(boneLeftLeg);
        bindRightUpLeg = Safe(boneRightUpLeg);
        bindRightLeg = Safe(boneRightLeg);

        bindSpineAxis = CaptureAxisLocal(boneSpine, boneSpine1, Vector3.up);
        bindSpine1Axis = CaptureAxisLocal(boneSpine1, boneSpine2, Vector3.up);
        bindSpine2Axis = CaptureAxisLocal(boneSpine2, boneNeck, Vector3.up);
        bindLeftArmAxis = CaptureAxisLocal(boneLeftArm, boneLeftForeArm, Vector3.right);
        bindLeftForeArmAxis = CaptureAxisLocal(boneLeftForeArm, boneLeftHand, Vector3.right);
        bindRightArmAxis = CaptureAxisLocal(boneRightArm, boneRightForeArm, Vector3.left);
        bindRightForeArmAxis = CaptureAxisLocal(boneRightForeArm, boneRightHand, Vector3.left);
        bindLeftUpLegAxis = CaptureAxisLocal(boneLeftUpLeg, boneLeftLeg, Vector3.down);
        bindLeftLegAxis = CaptureAxisLocal(boneLeftLeg, boneLeftFoot, Vector3.down);
        bindRightUpLegAxis = CaptureAxisLocal(boneRightUpLeg, boneRightLeg, Vector3.down);
        bindRightLegAxis = CaptureAxisLocal(boneRightLeg, boneRightFoot, Vector3.down);

        curHips = bindHips;
        curSpine = bindSpine;
        curSpine1 = bindSpine1;
        curSpine2 = bindSpine2;
        curLeftArm = bindLeftArm;
        curLeftForeArm = bindLeftForeArm;
        curRightArm = bindRightArm;
        curRightForeArm = bindRightForeArm;
        curLeftUpLeg = bindLeftUpLeg;
        curLeftLeg = bindLeftLeg;
        curRightUpLeg = bindRightUpLeg;
        curRightLeg = bindRightLeg;
    }

    private Quaternion Safe(Transform t) =>
        t != null ? t.localRotation : Quaternion.identity;

    private Vector3 CaptureAxisLocal(Transform bone, Transform child, Vector3 fallbackLocal)
    {
        if (bone == null || child == null) return fallbackLocal.normalized;

        Vector3 worldDir = child.position - bone.position;
        if (worldDir.sqrMagnitude < 0.0001f) return fallbackLocal.normalized;

        return bone.InverseTransformDirection(worldDir.normalized).normalized;
    }

    private void CaptureRootMotionReference()
    {
        motionRoot = transform;

        if (boneHips == null)
        {
            bindRootToHipOffset = Vector3.zero;
            return;
        }

        bindRootToHipOffset = boneHips.position - motionRoot.position;
        minHipHeightAboveFloor = Mathf.Max(minHipHeightAboveFloor, boneHips.position.y - LowestFootY());
    }

    private float LowestFootY()
    {
        float y = float.PositiveInfinity;
        if (boneLeftFoot != null) y = Mathf.Min(y, boneLeftFoot.position.y);
        if (boneRightFoot != null) y = Mathf.Min(y, boneRightFoot.position.y);
        return float.IsPositiveInfinity(y) && boneHips != null ? boneHips.position.y : y;
    }

    // ── UDP ───────────────────────────────────────────────────
    private void StartReceiver()
    {
        StopReceiver();
        try
        {
            running = true;
            udpClient = new UdpClient(listenPort);
            receiveThread = new Thread(ReceiveLoop);
            receiveThread.IsBackground = true;
            receiveThread.Start();
            Debug.Log($"PoseBoneDriver listening on :{listenPort}");
        }
        catch (Exception ex)
        {
            running = false;
            Debug.LogError($"PoseBoneDriver UDP failed: {ex.Message}");
        }
    }

    private void StopReceiver()
    {
        running = false;
        udpClient?.Close();
        udpClient = null;
        if (receiveThread != null && receiveThread.IsAlive)
            receiveThread.Join(300);
        receiveThread = null;
    }

    private void ReceiveLoop()
    {
        IPEndPoint ep = new IPEndPoint(IPAddress.Any, 0);
        while (running)
        {
            try
            {
                byte[] data = udpClient.Receive(ref ep);
                double receivedUnixMs = RuntimeMetricsRecorder.UnixMs();
                string receivedJson = Encoding.UTF8.GetString(data);
                lock (jsonLock)
                {
                    latestJson = receivedJson;
                    latestReceiveUnixMs = receivedUnixMs;
                    latestPacketBytes = data.Length;
                }
            }
            catch (ObjectDisposedException) { break; }
            catch (SocketException) { if (!running) break; }
            catch (Exception ex) { if (running) Debug.LogWarning($"UDP: {ex.Message}"); }
        }
    }

    // ── Update ────────────────────────────────────────────────
    private void Update()
    {
        string json = null;
        double receiveUnixMs = 0.0;
        int packetBytes = 0;
        lock (jsonLock)
        {
            if (!string.IsNullOrEmpty(latestJson))
            {
                json = latestJson;
                receiveUnixMs = latestReceiveUnixMs;
                packetBytes = latestPacketBytes;
                latestJson = null;
            }
        }

        if (!string.IsNullOrEmpty(json))
            ParsePacket(json, receiveUnixMs, packetBytes);

        DriveBones();
        UpdateOverlay();

        if (!Mathf.Approximately(jointSize, lastJointSize))
        {
            lastJointSize = jointSize;
            if (debugSpheres != null)
                foreach (var s in debugSpheres)
                    if (s != null) s.transform.localScale = Vector3.one * jointSize;
        }
    }

    private void ParsePacket(string json, double receiveUnixMs, int packetBytes)
    {
        PoseBonePacket2 packet;
        try { packet = JsonUtility.FromJson<PoseBonePacket2>(json); }
        catch { return; }

        if (packet == null || packet.type != "pose3d") return;
        if (packet.frame <= lastFrameId && lastFrameId - packet.frame < 1000) return;

        lastFrameId = packet.frame;
        RuntimeMetricsRecorder recorder = RuntimeMetricsRecorder.Instance;
        if (recorder != null)
            recorder.RecordPoseReceived(packet.frame, packet.metrics, receiveUnixMs, packetBytes);

        packetCoordinatesAreUnityWorld = packet.coordinate_space == "unity_world";
        UpdateAnchorMapping(packet);
        UpdateMappedCameras(packet);

        if (packet.people != null && packet.people.Length > 0)
        {
            if (spawnAvatarsForPeople && receiveUdp)
                HandlePeoplePacket(packet);
            else
                HandleSinglePersonPacket(packet);
        }
        else if (packet.joints != null)
        {
            ApplyJoints(packet.joints);
        }

        if (recorder != null)
            recorder.RecordPoseApplied(packet.frame);
    }

    private void ApplyJoints(JointPointBone2[] packetJoints)
    {
        for (int i = 0; i < 17; i++) active[i] = false;
        bool anyActiveJoint = false;

        foreach (JointPointBone2 j in packetJoints)
        {
            if (j == null || j.id < 0 || j.id >= 17) continue;
            if (!j.tracked || j.confidence < minConfidence) continue;
            joints[j.id] = MapIncomingPoint(new Vector3(j.x, j.y, j.z));
            active[j.id] = true;
            anyActiveJoint = true;
        }

        if (anyActiveJoint)
            LastPoseUpdateTime = Time.time;
    }

    private void HandlePeoplePacket(PoseBonePacket2 packet)
    {
        HashSet<int> visibleIds = new HashSet<int>();

        foreach (PersonPoseBone2 person in packet.people)
        {
            if (person == null || !person.tracked || person.joints == null)
                continue;

            int personId = person.id;
            if (personId <= 0)
                personId = 1;

            PoseBoneDriver driver = GetOrCreateAvatarDriver(personId);
            if (driver == null)
                continue;

            visibleIds.Add(personId);
            avatarLastSeen[personId] = Time.time;
            driver.packetCoordinatesAreUnityWorld = packetCoordinatesAreUnityWorld;
            driver.ApplyJoints(person.joints);
        }

        RemoveStaleAvatars(visibleIds);
    }

    private PoseBoneDriver GetOrCreateAvatarDriver(int personId)
    {
        if (avatarDrivers.TryGetValue(personId, out PoseBoneDriver existing) && existing != null)
            return existing;

        if (assignedPersonId < 0 || assignedPersonId == personId)
        {
            assignedPersonId = personId;
            avatarDrivers[personId] = this;
            gameObject.name = "character_person_" + personId;
            return this;
        }

        GameObject cloneObject = Instantiate(gameObject, transform.parent);
        cloneObject.name = "character_person_" + personId;
        PoseBoneDriver clone = cloneObject.GetComponent<PoseBoneDriver>();
        if (clone == null)
            return null;

        clone.receiveUdp = false;
        clone.spawnAvatarsForPeople = false;
        clone.isSpawnedAvatar = true;
        clone.assignedPersonId = personId;
        clone.avatarDrivers.Clear();
        clone.avatarLastSeen.Clear();

        avatarDrivers[personId] = clone;
        return clone;
    }

    private void RemoveStaleAvatars(HashSet<int> visibleIds)
    {
        List<int> staleIds = new List<int>();
        foreach (KeyValuePair<int, PoseBoneDriver> entry in avatarDrivers)
        {
            int personId = entry.Key;
            if (visibleIds.Contains(personId))
                continue;

            float lastSeen = avatarLastSeen.ContainsKey(personId) ? avatarLastSeen[personId] : 0f;
            if (Time.time - lastSeen >= avatarDisappearSeconds)
                staleIds.Add(personId);
        }

        foreach (int personId in staleIds)
        {
            PoseBoneDriver driver = avatarDrivers[personId];
            avatarDrivers.Remove(personId);
            avatarLastSeen.Remove(personId);

            if (driver == this)
            {
                assignedPersonId = -1;
                rootPositionInitialized = false;
                for (int i = 0; i < active.Length; i++) active[i] = false;
            }
            else if (driver != null)
            {
                Destroy(driver.gameObject);
            }
        }
    }

    // ── Drive bones ───────────────────────────────────────────
    private void DriveBones()
    {
        float dt = Time.deltaTime;
        float s = smoothing > 0f ? Mathf.Clamp01(dt * smoothing) : 1f;
        UpdateGroundOffset(s);

        // ── Hips position + rotation ──────────────────────────
        if (lockRootRotation && motionRoot != null)
            motionRoot.rotation = Quaternion.identity;

        if (active[LEFT_HIP] && active[RIGHT_HIP] && boneHips != null)
        {
            Vector3 leftHip = Joint(LEFT_HIP);
            Vector3 rightHip = Joint(RIGHT_HIP);
            Vector3 hipMid = (leftHip + rightHip) * 0.5f;
            hipMid = ClampHipToFloor(hipMid);

            if (driveRootXZ)
                DriveRootXZ(hipMid, s);

            Vector3 hipRight = (rightHip - leftHip).normalized;
            Vector3 hipUp = Vector3.up;

            if (active[LEFT_SHOULDER] && active[RIGHT_SHOULDER])
            {
                Vector3 shoulderMid = (Joint(LEFT_SHOULDER) + Joint(RIGHT_SHOULDER)) * 0.5f;
                hipUp = (shoulderMid - hipMid).normalized;
            }

            Vector3 hipFwd = Vector3.Cross(hipRight, hipUp).normalized;
            if (driveRootYaw && !lockRootRotation)
                DriveRootYaw(hipFwd, s);

            if (driveHipsRotation && hipFwd.sqrMagnitude > 0.001f)
            {
                Quaternion targetWorld = Quaternion.LookRotation(hipFwd, hipUp);
                Quaternion parentRot = boneHips.parent != null
                    ? boneHips.parent.rotation : Quaternion.identity;
                Quaternion targetLocal = Quaternion.Inverse(parentRot) * targetWorld;

                curHips = Quaternion.Slerp(curHips, targetLocal, s);
                boneHips.localRotation = curHips;
            }
            else
            {
                curHips = Quaternion.Slerp(curHips, bindHips, s);
                boneHips.localRotation = curHips;
            }

            boneHips.position = Vector3.Lerp(boneHips.position, hipMid, s);
        }

        // ── Spine ─────────────────────────────────────────────
        if (active[LEFT_SHOULDER] && active[RIGHT_SHOULDER] &&
            active[LEFT_HIP] && active[RIGHT_HIP] && driveSpineRotation)
        {
            Vector3 hipMid = (Joint(LEFT_HIP) + Joint(RIGHT_HIP)) * 0.5f;
            Vector3 shoulderMid = (Joint(LEFT_SHOULDER) + Joint(RIGHT_SHOULDER)) * 0.5f;

            DriveSegmentAlongBindAxis(hipMid, shoulderMid, true,
                boneSpine, bindSpine, bindSpineAxis, ref curSpine, s);

            DriveSegmentAlongBindAxis(hipMid, shoulderMid, true,
                boneSpine1, bindSpine1, bindSpine1Axis, ref curSpine1, s);

            DriveSegmentAlongBindAxis(hipMid, shoulderMid, true,
                boneSpine2, bindSpine2, bindSpine2Axis, ref curSpine2, s);
        }
        else
        {
            ResetSpineToBind(s);
        }

        Vector3 leftShoulder = Joint(LEFT_SHOULDER);
        Vector3 leftElbow = Joint(LEFT_ELBOW);
        Vector3 leftWrist = Joint(LEFT_WRIST);
        Vector3 rightShoulder = Joint(RIGHT_SHOULDER);
        Vector3 rightElbow = Joint(RIGHT_ELBOW);
        Vector3 rightWrist = Joint(RIGHT_WRIST);

        if (stabilizeArmDepth)
        {
            leftElbow = StabilizeDepth(leftShoulder, leftElbow, armDepthWeight);
            leftWrist = StabilizeDepth(leftElbow, leftWrist, armDepthWeight);
            rightElbow = StabilizeDepth(rightShoulder, rightElbow, armDepthWeight);
            rightWrist = StabilizeDepth(rightElbow, rightWrist, armDepthWeight);
        }

        // Arms align their real bind-pose axes to the detected arm segments.
        DriveSegmentAlongBindAxis(leftShoulder, leftElbow,
            active[LEFT_SHOULDER] && active[LEFT_ELBOW],
            boneLeftArm, bindLeftArm, bindLeftArmAxis, ref curLeftArm, s);

        DriveSegmentAlongBindAxis(leftElbow, leftWrist,
            active[LEFT_ELBOW] && active[LEFT_WRIST],
            boneLeftForeArm, bindLeftForeArm, bindLeftForeArmAxis, ref curLeftForeArm, s);

        DriveSegmentAlongBindAxis(rightShoulder, rightElbow,
            active[RIGHT_SHOULDER] && active[RIGHT_ELBOW],
            boneRightArm, bindRightArm, bindRightArmAxis, ref curRightArm, s);

        DriveSegmentAlongBindAxis(rightElbow, rightWrist,
            active[RIGHT_ELBOW] && active[RIGHT_WRIST],
            boneRightForeArm, bindRightForeArm, bindRightForeArmAxis, ref curRightForeArm, s);

        Vector3 leftLegHip = Joint(LEFT_HIP);
        Vector3 leftLegKnee = Joint(LEFT_KNEE);
        Vector3 leftLegAnkle = Joint(LEFT_ANKLE);
        Vector3 rightLegHip = Joint(RIGHT_HIP);
        Vector3 rightLegKnee = Joint(RIGHT_KNEE);
        Vector3 rightLegAnkle = Joint(RIGHT_ANKLE);

        if (stabilizeLegDepth)
        {
            leftLegKnee = StabilizeMiddleDepth(leftLegHip, leftLegKnee, leftLegAnkle, legDepthWeight);
            rightLegKnee = StabilizeMiddleDepth(rightLegHip, rightLegKnee, rightLegAnkle, legDepthWeight);
        }

        if (active[LEFT_HIP] && active[LEFT_KNEE] && active[LEFT_ANKLE])
            leftLegKnee = CorrectKneeBend(leftLegHip, leftLegKnee, leftLegAnkle, flipLeftLegBend);

        if (active[RIGHT_HIP] && active[RIGHT_KNEE] && active[RIGHT_ANKLE])
            rightLegKnee = CorrectKneeBend(rightLegHip, rightLegKnee, rightLegAnkle, flipRightLegBend);

        // Legs align their bind-pose bone axis to the target segment direction.
        // The knee correction above mirrors backward knee bends without moving feet.
        DriveSegmentAlongBindAxis(leftLegHip, leftLegKnee,
            active[LEFT_HIP] && active[LEFT_KNEE],
            boneLeftUpLeg, bindLeftUpLeg, bindLeftUpLegAxis, ref curLeftUpLeg, s);

        DriveSegmentAlongBindAxis(leftLegKnee, leftLegAnkle,
            active[LEFT_KNEE] && active[LEFT_ANKLE],
            boneLeftLeg, bindLeftLeg, bindLeftLegAxis, ref curLeftLeg, s);

        DriveSegmentAlongBindAxis(rightLegHip, rightLegKnee,
            active[RIGHT_HIP] && active[RIGHT_KNEE],
            boneRightUpLeg, bindRightUpLeg, bindRightUpLegAxis, ref curRightUpLeg, s);

        DriveSegmentAlongBindAxis(rightLegKnee, rightLegAnkle,
            active[RIGHT_KNEE] && active[RIGHT_ANKLE],
            boneRightLeg, bindRightLeg, bindRightLegAxis, ref curRightLeg, s);
    }

    private void ResetSpineToBind(float s)
    {
        curSpine = Quaternion.Slerp(curSpine, bindSpine, s);
        curSpine1 = Quaternion.Slerp(curSpine1, bindSpine1, s);
        curSpine2 = Quaternion.Slerp(curSpine2, bindSpine2, s);

        if (boneSpine != null) boneSpine.localRotation = curSpine;
        if (boneSpine1 != null) boneSpine1.localRotation = curSpine1;
        if (boneSpine2 != null) boneSpine2.localRotation = curSpine2;
    }

    private Vector3 CorrectKneeBend(Vector3 hip, Vector3 knee, Vector3 ankle, bool flip)
    {
        if (!flip) return knee;

        Vector3 hipToAnkle = ankle - hip;
        if (hipToAnkle.sqrMagnitude < 0.0001f) return knee;

        Vector3 axis = hipToAnkle.normalized;
        Vector3 hipToKnee = knee - hip;
        Vector3 alongAxis = Vector3.Project(hipToKnee, axis);
        Vector3 bend = hipToKnee - alongAxis;
        if (bend.sqrMagnitude < 0.0001f) return knee;

        return hip + alongAxis - bend;
    }

    private Vector3 StabilizeDepth(Vector3 anchor, Vector3 point, float depthWeight)
    {
        point.z = Mathf.Lerp(anchor.z, point.z, Mathf.Clamp01(depthWeight));
        return point;
    }

    private Vector3 StabilizeMiddleDepth(Vector3 start, Vector3 middle, Vector3 end, float depthWeight)
    {
        float planeZ = (start.z + end.z) * 0.5f;
        middle.z = Mathf.Lerp(planeZ, middle.z, Mathf.Clamp01(depthWeight));
        return middle;
    }

    private Vector3 Joint(int id) => joints[id] + groundOffset;

    public bool TryGetJointWorldPosition(int id, out Vector3 position)
    {
        position = Vector3.zero;
        if (joints == null || active == null || id < 0 || id >= joints.Length || id >= active.Length || !active[id])
            return false;

        position = Joint(id);
        return IsFinite(position);
    }

    public Vector3[] GetActiveJointWorldPositions()
    {
        List<Vector3> points = new List<Vector3>();
        CopyActiveJointWorldPositions(points);
        return points.ToArray();
    }

    public int CopyActiveJointWorldPositions(List<Vector3> target)
    {
        if (target == null || joints == null || active == null)
            return 0;

        int copied = 0;
        int count = Mathf.Min(joints.Length, active.Length);
        for (int i = 0; i < count; i++)
        {
            if (!active[i])
                continue;

            Vector3 point = Joint(i);
            if (!IsFinite(point))
                continue;

            target.Add(point);
            copied++;
        }

        return copied;
    }

    private static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private Vector3 ClampHipToFloor(Vector3 hipMid)
    {
        float minY = floorY + minHipHeightAboveFloor;
        if (hipMid.y < minY) hipMid.y = minY;
        return hipMid;
    }

    private void DriveRootXZ(Vector3 hipMid, float s)
    {
        if (motionRoot == null) return;

        Vector3 target = motionRoot.position;
        target.x = hipMid.x - bindRootToHipOffset.x;
        target.z = hipMid.z - bindRootToHipOffset.z;

        Vector3 delta = target - motionRoot.position;
        delta.y = 0f;

        if (!rootPositionInitialized)
        {
            motionRoot.position = target;
            rootPositionInitialized = true;
            return;
        }

        if (delta.magnitude > maxRootSnapDistance)
            return;

        float maxStep = Mathf.Max(0.01f, maxRootSpeed) * Time.deltaTime;
        if (delta.magnitude > maxStep)
            target = motionRoot.position + delta.normalized * maxStep;

        motionRoot.position = Vector3.Lerp(motionRoot.position, target, s);
    }

    private void DriveRootYaw(Vector3 forward, float s)
    {
        if (motionRoot == null)
            return;

        forward.y = 0f;
        if (forward.sqrMagnitude < 0.0001f)
            return;

        Quaternion target = Quaternion.LookRotation(forward.normalized, Vector3.up) *
            Quaternion.Euler(0f, rootYawOffsetDegrees, 0f);
        motionRoot.rotation = Quaternion.Slerp(motionRoot.rotation, target, s);
    }

    private void UpdateGroundOffset(float s)
    {
        if (!groundToAnkles)
        {
            groundOffset = Vector3.Lerp(groundOffset, Vector3.zero, s);
            return;
        }

        bool hasAnkle = false;
        float ankleY = float.PositiveInfinity;

        if (active[LEFT_ANKLE])
        {
            ankleY = Mathf.Min(ankleY, joints[LEFT_ANKLE].y);
            hasAnkle = true;
        }

        if (active[RIGHT_ANKLE])
        {
            ankleY = Mathf.Min(ankleY, joints[RIGHT_ANKLE].y);
            hasAnkle = true;
        }

        if (!hasAnkle) return;

        float correction = Mathf.Clamp(floorY - ankleY, -maxGroundCorrection, maxGroundCorrection);
        Vector3 targetOffset = new Vector3(0f, correction, 0f);
        groundOffset = Vector3.Lerp(groundOffset, targetOffset, s);
    }

    private void DriveSegment(
        Vector3 from, Vector3 to, bool valid,
        Transform bone, Quaternion bindLocal,
        ref Quaternion current, float s,
        Vector3 upHint)
    {
        if (!valid || bone == null) return;

        Vector3 dir = (to - from).normalized;
        if (dir.sqrMagnitude < 0.0001f) return;

        // Avoid gimbal — if dir is parallel to upHint, use fallback
        Vector3 up = (Mathf.Abs(Vector3.Dot(dir, upHint)) > 0.95f)
            ? (upHint == Vector3.up ? Vector3.forward : Vector3.up)
            : upHint;

        Quaternion targetWorld = Quaternion.LookRotation(dir, up);

        // Apply rotation offset
        if (rotationOffsetEuler != Vector3.zero)
            targetWorld = targetWorld * Quaternion.Euler(rotationOffsetEuler);

        Quaternion parentWorld = bone.parent != null
            ? bone.parent.rotation : Quaternion.identity;

        Quaternion bindWorld = parentWorld * bindLocal;
        Quaternion delta = targetWorld * Quaternion.Inverse(bindWorld);
        Quaternion targetLocal = delta * bindLocal;

        current = Quaternion.Slerp(current, targetLocal, s);
        bone.localRotation = current;
    }

    private void DriveSegmentAlongBindAxis(
        Vector3 from, Vector3 to, bool valid,
        Transform bone, Quaternion bindLocal, Vector3 bindAxisLocal,
        ref Quaternion current, float s)
    {
        if (!valid || bone == null) return;

        Vector3 targetDir = (to - from).normalized;
        if (targetDir.sqrMagnitude < 0.0001f) return;

        Quaternion parentWorld = bone.parent != null
            ? bone.parent.rotation : Quaternion.identity;

        Quaternion bindWorld = parentWorld * bindLocal;
        Vector3 bindAxisWorld = (bindWorld * bindAxisLocal).normalized;
        if (bindAxisWorld.sqrMagnitude < 0.0001f) return;

        Quaternion deltaWorld = Quaternion.FromToRotation(bindAxisWorld, targetDir);
        Quaternion targetWorld = deltaWorld * bindWorld;
        Quaternion targetLocal = Quaternion.Inverse(parentWorld) * targetWorld;

        current = Quaternion.Slerp(current, targetLocal, s);
        bone.localRotation = current;
    }

    // ── Coordinate conversion ─────────────────────────────────
    private Vector3 Convert(Vector3 p)
    {
        if (swapXZ)
        {
            float x = p.x;
            p.x = p.z;
            p.z = x;
        }

        if (mirrorX) p.x = -p.x;
        if (invertY) p.y = -p.y;
        if (invertZ) p.z = -p.z;
        p *= poseScale;
        p = Quaternion.Euler(rotationOffsetEuler) * p;
        p += poseOffset;
        return p;
    }

    private Vector3 RawIncoming(ArucoMarkerPointBone2 marker)
    {
        return new Vector3(marker.x, marker.y, marker.z);
    }

    private Vector3 RawIncoming(CameraPointBone2 cameraPoint)
    {
        return new Vector3(cameraPoint.x, cameraPoint.y, cameraPoint.z);
    }

    private Vector3 MapIncomingPoint(Vector3 incoming)
    {
        if (packetCoordinatesAreUnityWorld)
            return incoming;

        if (!useArucoAnchors || !anchorMappingValid)
            return Convert(incoming);

        Vector3 relative = incoming - anchorRealCentroid;
        return anchorUnityCentroid + (anchorRotation * (relative * anchorScale)) + anchorMappedOffset;
    }

    private Vector3 MapIncomingDirection(Vector3 incoming)
    {
        if (!useArucoAnchors || !anchorMappingValid)
            return Quaternion.Euler(rotationOffsetEuler) * incoming;

        return anchorRotation * incoming;
    }

    private void AutoFindAnchorObjects()
    {
        if (!autoFindArucoAnchors)
            return;

        if (arucoAnchors == null || arucoAnchors.Length < 50)
            Array.Resize(ref arucoAnchors, 50);

        for (int id = 0; id < arucoAnchors.Length; id++)
        {
            if (arucoAnchors[id] == null)
                arucoAnchors[id] = FindArucoAnchorTransform(id);
        }

        if (mappedCamera1 == null)
        {
            GameObject cam1 = GameObject.Find("YoloPose_Calibration_Rig/RealRobot_Camera_1_Cam1Origin");
            if (cam1 != null) mappedCamera1 = cam1.transform;
        }

        if (mappedCamera2 == null)
        {
            GameObject cam2 = GameObject.Find("YoloPose_Calibration_Rig/RealRobot_Camera_2_StereoRelative");
            if (cam2 != null) mappedCamera2 = cam2.transform;
        }
    }

    private Transform FindArucoAnchorTransform(int id)
    {
        string padded = arucoMarkerNamePrefix + id.ToString("00");
        string plain = arucoMarkerNamePrefix + id.ToString();

        Transform[] transforms = Resources.FindObjectsOfTypeAll<Transform>();
        foreach (Transform t in transforms)
        {
            if (t == null || !t.gameObject.scene.IsValid())
                continue;

            string n = t.name;
            if (n == plain || n.StartsWith(padded, StringComparison.Ordinal))
                return t;
        }

        return null;
    }

    private void UpdateAnchorMapping(PoseBonePacket2 packet)
    {
        anchorMappingValid = false;

        if (!useArucoAnchors || packet == null || packet.aruco_markers == null)
            return;

        if (autoFindArucoAnchors)
            AutoFindAnchorObjects();

        List<Vector3> realPoints = new List<Vector3>();
        List<Vector3> unityPoints = new List<Vector3>();

        foreach (ArucoMarkerPointBone2 marker in packet.aruco_markers)
        {
            if (marker == null || !marker.tracked || marker.confidence < minConfidence)
                continue;
            if (marker.id < 0 || arucoAnchors == null || marker.id >= arucoAnchors.Length)
                continue;

            Transform anchor = arucoAnchors[marker.id];
            if (anchor == null)
                continue;

            realPoints.Add(RawIncoming(marker));
            unityPoints.Add(anchor.position);
        }

        int count = realPoints.Count;
        if (count == 0)
            return;

        anchorRealCentroid = Vector3.zero;
        anchorUnityCentroid = Vector3.zero;
        for (int i = 0; i < count; i++)
        {
            anchorRealCentroid += realPoints[i];
            anchorUnityCentroid += unityPoints[i];
        }
        anchorRealCentroid /= count;
        anchorUnityCentroid /= count;

        if (count == 1)
        {
            anchorRotation = Quaternion.identity;
            anchorScale = Mathf.Clamp(singleAnchorScale, minAnchorScale, maxAnchorScale);
            anchorMappingValid = true;
            return;
        }

        if (count == 2)
        {
            Vector3 realAxis = realPoints[1] - realPoints[0];
            Vector3 unityAxis = unityPoints[1] - unityPoints[0];
            float realDistance = realAxis.magnitude;
            float unityDistance = unityAxis.magnitude;

            if (realDistance <= 0.0001f || unityDistance <= 0.0001f)
            {
                anchorRotation = Quaternion.identity;
                anchorScale = Mathf.Clamp(singleAnchorScale, minAnchorScale, maxAnchorScale);
                anchorMappingValid = true;
                return;
            }

            anchorRotation = Quaternion.FromToRotation(realAxis.normalized, unityAxis.normalized);
            anchorScale = Mathf.Clamp(unityDistance / realDistance, minAnchorScale, maxAnchorScale);
            anchorMappingValid = true;
            return;
        }

        Vector3 realRight, realUp, realForward;
        Vector3 unityRight, unityUp, unityForward;
        int basisA, basisB, basisC;
        if (!TryChooseAnchorBasisIndices(realPoints, out basisA, out basisB, out basisC) ||
            !TryBuildAnchorBasis(realPoints, basisA, basisB, basisC, out realRight, out realUp, out realForward) ||
            !TryBuildAnchorBasis(unityPoints, basisA, basisB, basisC, out unityRight, out unityUp, out unityForward))
        {
            anchorRotation = Quaternion.identity;
            anchorScale = Mathf.Clamp(singleAnchorScale, minAnchorScale, maxAnchorScale);
            anchorMappingValid = true;
            return;
        }

        Quaternion realBasis = Quaternion.LookRotation(realForward, realUp);
        Quaternion unityBasis = Quaternion.LookRotation(unityForward, unityUp);
        anchorRotation = unityBasis * Quaternion.Inverse(realBasis);

        float numerator = 0f;
        float denominator = 0f;
        for (int i = 0; i < count; i++)
        {
            Vector3 realRelative = realPoints[i] - anchorRealCentroid;
            Vector3 unityRelative = unityPoints[i] - anchorUnityCentroid;
            Vector3 rotatedReal = anchorRotation * realRelative;
            numerator += Vector3.Dot(unityRelative, rotatedReal);
            denominator += Vector3.Dot(realRelative, realRelative);
        }

        if (denominator <= 0.000001f)
        {
            anchorScale = Mathf.Clamp(singleAnchorScale, minAnchorScale, maxAnchorScale);
        }
        else
        {
            anchorScale = Mathf.Clamp(numerator / denominator, minAnchorScale, maxAnchorScale);
        }

        anchorMappingValid = true;
    }

    private bool TryChooseAnchorBasisIndices(List<Vector3> points, out int a, out int b, out int c)
    {
        a = 0;
        b = 1;
        c = -1;

        if (points == null || points.Count < 3)
            return false;

        float bestDistanceSqr = 0f;
        for (int i = 0; i < points.Count; i++)
        {
            for (int j = i + 1; j < points.Count; j++)
            {
                float distanceSqr = (points[j] - points[i]).sqrMagnitude;
                if (distanceSqr > bestDistanceSqr)
                {
                    bestDistanceSqr = distanceSqr;
                    a = i;
                    b = j;
                }
            }
        }

        if (bestDistanceSqr <= 0.000001f)
            return false;

        Vector3 right = (points[b] - points[a]).normalized;
        float bestAreaSqr = 0f;
        for (int i = 0; i < points.Count; i++)
        {
            if (i == a || i == b)
                continue;

            Vector3 candidate = Vector3.Cross(right, points[i] - points[a]);
            float areaSqr = candidate.sqrMagnitude;
            if (areaSqr > bestAreaSqr)
            {
                bestAreaSqr = areaSqr;
                c = i;
            }
        }

        if (c < 0 || bestAreaSqr <= 0.000001f)
            return false;

        return true;
    }

    private bool TryBuildAnchorBasis(List<Vector3> points, int a, int b, int c, out Vector3 right, out Vector3 up, out Vector3 forward)
    {
        right = Vector3.right;
        up = Vector3.up;
        forward = Vector3.forward;

        if (points == null || a < 0 || b < 0 || c < 0 ||
            a >= points.Count || b >= points.Count || c >= points.Count)
            return false;

        Vector3 axis = points[b] - points[a];
        if (axis.sqrMagnitude <= 0.000001f)
            return false;

        right = axis.normalized;
        forward = Vector3.Cross(right, points[c] - points[a]).normalized;
        if (forward.sqrMagnitude <= 0.000001f)
            return false;

        up = Vector3.Cross(forward, right).normalized;
        return true;
    }

    private void UpdateMappedCameras(PoseBonePacket2 packet)
    {
        if (!updateMappedCameraObjects || !useArucoAnchors || !anchorMappingValid ||
            packet == null || packet.cameras == null)
            return;

        foreach (CameraPointBone2 cameraPoint in packet.cameras)
        {
            if (cameraPoint == null || !cameraPoint.tracked)
                continue;

            Transform target = null;
            if (cameraPoint.id == 1 || cameraPoint.name == "cam1")
                target = mappedCamera1;
            else if (cameraPoint.id == 2 || cameraPoint.name == "cam2")
                target = mappedCamera2;

            if (target == null)
                continue;

            target.position = MapIncomingPoint(RawIncoming(cameraPoint));
            if (cameraPoint.hasOrientation)
            {
                Vector3 forward = MapIncomingDirection(new Vector3(
                    cameraPoint.forwardX,
                    cameraPoint.forwardY,
                    cameraPoint.forwardZ
                ));
                Vector3 up = MapIncomingDirection(new Vector3(
                    cameraPoint.upX,
                    cameraPoint.upY,
                    cameraPoint.upZ
                ));

                if (forward.sqrMagnitude > 0.0001f && up.sqrMagnitude > 0.0001f)
                    target.rotation = Quaternion.LookRotation(forward.normalized, up.normalized);
            }
            else
            {
                Vector3 lookTarget = anchorUnityCentroid;
                if ((lookTarget - target.position).sqrMagnitude > 0.0001f)
                    target.LookAt(lookTarget, Vector3.up);
            }
        }
    }

    // ── Skeleton overlay ──────────────────────────────────────
    private void CreateOverlay()
    {
        debugSpheres = new GameObject[17];
        lastJointSize = jointSize;

        for (int i = 0; i < 17; i++)
        {
            GameObject s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            s.name = $"SkelJoint_{i}";
            s.transform.localScale = Vector3.one * jointSize;
            var mat = new Material(Shader.Find("Sprites/Default"));
            mat.color = jointColor;
            s.GetComponent<Renderer>().material = mat;
            Destroy(s.GetComponent<Collider>());
            s.SetActive(false);
            debugSpheres[i] = s;
        }

        debugLines = new LineRenderer[skeletonDef.GetLength(0)];
        for (int i = 0; i < skeletonDef.GetLength(0); i++)
        {
            GameObject lo = new GameObject($"SkelBone_{i}");
            LineRenderer lr = lo.AddComponent<LineRenderer>();
            lr.positionCount = 2;
            lr.startWidth = boneWidth;
            lr.endWidth = boneWidth;
            lr.useWorldSpace = true;
            var mat = new Material(Shader.Find("Sprites/Default"));
            mat.color = boneColor;
            lr.material = mat;
            lr.startColor = boneColor;
            lr.endColor = boneColor;
            lo.SetActive(false);
            debugLines[i] = lr;
        }
    }

    private void UpdateOverlay()
    {
        if (debugSpheres == null) return;

        for (int i = 0; i < 17; i++)
        {
            bool show = showSkeleton && active[i];
            debugSpheres[i].SetActive(show);
            if (show) debugSpheres[i].transform.position = Joint(i);
        }

        for (int i = 0; i < skeletonDef.GetLength(0); i++)
        {
            int a = skeletonDef[i, 0];
            int b = skeletonDef[i, 1];
            bool show = showSkeleton && active[a] && active[b];
            debugLines[i].gameObject.SetActive(show);
            if (show)
            {
                debugLines[i].SetPosition(0, Joint(a));
                debugLines[i].SetPosition(1, Joint(b));
            }
        }
    }


    private void HandleSinglePersonPacket(PoseBonePacket2 packet)
    {
        foreach (PersonPoseBone2 person in packet.people)
        {
            if (person == null || !person.tracked || person.joints == null)
                continue;

            assignedPersonId = person.id > 0 ? person.id : 1;
            ApplyJoints(person.joints);
            return;
        }
    }
}
