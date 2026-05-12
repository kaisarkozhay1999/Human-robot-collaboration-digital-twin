using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Newtonsoft.Json.Linq;

public class PoseReceiver : MonoBehaviour
{
    UdpClient client;

    public Animator animator;
    public Transform avatarRoot;

    Vector3 lHip, rHip;
    Vector3 lShoulder, rShoulder;

    Vector3 previousHip;
    bool initialized = false;

    void Start()
    {
        client = new UdpClient(5055);
        client.Client.Blocking = false;

        Debug.Log("Walk + Turn System Ready");
    }

    void Update()
    {
        ReceiveData();

        Vector3 midHip = (lHip + rHip) * 0.5f;

        if (!initialized)
        {
            previousHip = midHip;
            initialized = true;
        }

        // -------------------------
        // SPEED DETECTION
        // -------------------------

        float rawSpeed = Vector3.Distance(midHip, previousHip) * 25f;
        previousHip = midHip;

        float smoothSpeed = Mathf.Lerp(
            animator.GetFloat("Speed"),
            rawSpeed,
            0.3f
        );

        animator.SetFloat("Speed", smoothSpeed);

        // -------------------------
        // ROTATION FROM SHOULDERS
        // -------------------------

        Vector3 shoulderDir = rShoulder - lShoulder;

        // Convert MediaPipe → Unity axes
        shoulderDir = new Vector3(-shoulderDir.x, 0, -shoulderDir.z);

        if (shoulderDir.magnitude > 0.01f)
        {
            Quaternion targetRotation =
                Quaternion.LookRotation(shoulderDir.normalized);

            avatarRoot.rotation =
                Quaternion.Slerp(
                    avatarRoot.rotation,
                    targetRotation,
                    4f * Time.deltaTime
                );
        }

        // -------------------------
        // FORWARD MOVEMENT
        // -------------------------

        if (smoothSpeed > 0.05f)
        {
            float moveSpeed = smoothSpeed * 2f;

            avatarRoot.position +=
                avatarRoot.forward * moveSpeed * Time.deltaTime;
        }
    }

    void ReceiveData()
    {
        if (client.Available <= 0)
            return;

        IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 5055);
        byte[] data = client.Receive(ref anyIP);
        string json = Encoding.UTF8.GetString(data);

        JObject obj = JObject.Parse(json);

        lHip = GetVec(obj["l_hip"]);
        rHip = GetVec(obj["r_hip"]);

        lShoulder = GetVec(obj["l_shoulder"]);
        rShoulder = GetVec(obj["r_shoulder"]);
    }

    Vector3 GetVec(JToken token)
    {
        return new Vector3(
            (float)token[0],
            (float)token[1],
            (float)token[2]
        );
    }
}