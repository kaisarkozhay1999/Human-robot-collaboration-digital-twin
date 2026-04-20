using UnityEngine;
using TMPro;
using System.Collections;

/// <summary>
/// Toggleable stress analysis heat map for the ER9 Pro robot arm.
/// 
/// Attach to the same GameObject as ER9ProFullController.
/// Assign the joint renderers and the toggle button label in the Inspector.
/// 
/// Physics model:
///   For each joint i, static torque = sum of (mass_j * g * horizontal_distance_j)
///   for all links j at or beyond joint i.
///   Torque is normalised against each joint's known max rated torque,
///   giving a 0-1 stress ratio that drives the green->yellow->red color map.
/// </summary>
public class StressAnalysisController : MonoBehaviour
{
    // ─────────────────────────────────────────────
    // Inspector
    // ─────────────────────────────────────────────

    [Header("Joint Renderers (assign in order: base → roll)")]
    public Renderer joint1Renderer;
    public Renderer joint2Renderer;
    public Renderer joint3Renderer;
    public Renderer joint4Renderer;
    public Renderer joint5Renderer;
    public Renderer gripperRenderer;

    [Header("UI")]
    public TMP_Text stressButtonLabel;

    [Header("Stress Labels (optional — floating text near each joint)")]
    public TMP_Text stressLabel1;
    public TMP_Text stressLabel2;
    public TMP_Text stressLabel3;
    public TMP_Text stressLabel4;
    public TMP_Text stressLabel5;

    [Header("ER9 Pro Physical Parameters")]
    [Tooltip("Mass of each link in kg: [base, upper arm, forearm, wrist, end effector]")]
    public float[] linkMass = { 0.8f, 0.6f, 0.4f, 0.2f, 0.1f };

    [Tooltip("Length of each link in meters: [upper arm, forearm, wrist, end effector]")]
    public float[] linkLength = { 0.2f, 0.17f, 0.10f, 0.07f };

    [Tooltip("Max rated torque per joint in Nm: [J1, J2, J3, J4, J5]")]
    public float[] maxTorque = { 15f, 12f, 8f, 4f, 2f };

    [Header("Color Map")]
    public Color colorLow    = Color.green;
    public Color colorMid    = Color.yellow;
    public Color colorHigh   = Color.red;
    public Color colorNormal = Color.white;

    [Header("Pulse animation on high stress")]
    [Tooltip("Stress ratio above which the joint pulses")]
    public float pulseThreshold = 0.75f;
    public float pulseSpeed     = 3f;

    // ─────────────────────────────────────────────
    // Private State
    // ─────────────────────────────────────────────

    bool   _stressMode  = false;
    float[] _stressRatio = new float[5];

    Renderer[]  _renderers;
    TMP_Text[]  _labels;
    Material[]  _originalMaterials;
    Material[]  _stressMaterials;

    ER9ProFullController _robot;

    static readonly int ColorProp = Shader.PropertyToID("_Color");

    // ─────────────────────────────────────────────
    // Unity Lifecycle
    // ─────────────────────────────────────────────

    void Awake()
    {
        _robot = GetComponent<ER9ProFullController>();

        _renderers = new Renderer[]
        {
            joint1Renderer, joint2Renderer, joint3Renderer,
            joint4Renderer, joint5Renderer, gripperRenderer
        };

        _labels = new TMP_Text[]
        {
            stressLabel1, stressLabel2, stressLabel3,
            stressLabel4, stressLabel5
        };

        // Cache original materials and create per-instance stress materials
        _originalMaterials = new Material[_renderers.Length];
        _stressMaterials   = new Material[_renderers.Length];

        for (int i = 0; i < _renderers.Length; i++)
        {
            if (_renderers[i] == null) continue;
            _originalMaterials[i] = _renderers[i].material;
            // Instance copy so we don't modify the shared asset
            _stressMaterials[i]   = new Material(_renderers[i].material);
        }
    }

    void Update()
    {
        if (!_stressMode) return;

        CalculateStress();
        ApplyHeatMap();
        UpdateStressLabels();
    }

    void OnDestroy()
    {
        // Clean up instanced materials
        if (_stressMaterials != null)
            foreach (var m in _stressMaterials)
                if (m != null) Destroy(m);
    }

    // ─────────────────────────────────────────────
    // Toggle
    // ─────────────────────────────────────────────

    public void ToggleStressAnalysis()
    {
        _stressMode = !_stressMode;

        if (stressButtonLabel != null)
            stressButtonLabel.text = _stressMode ? "Normal View" : "Stress View";

        if (_stressMode)
        {
            EnableStressMaterials();
        }
        else
        {
            RestoreOriginalMaterials();
            HideStressLabels();
        }
    }

    // ─────────────────────────────────────────────
    // Physics — Static Torque Model
    // ─────────────────────────────────────────────

    /// <summary>
    /// Calculates normalised stress ratio (0–1) for each joint using
    /// a simplified planar static torque model.
    ///
    /// For joint i, torque = sum over all distal links j of:
    ///     mass[j] * g * horizontal_reach[j]
    ///
    /// Horizontal reach of link j depends on the cumulative joint angles
    /// from joint i+1 outward.
    /// </summary>
    void CalculateStress()
    {
        // Read current smoothed joint angles from the robot controller
        // a1=base(yaw), a2=shoulder, a3=elbow, a4=pitch, a5=roll
        float a2 = _robot != null ? GetJointAngle(1) : 0f;  // shoulder (deg)
        float a3 = _robot != null ? GetJointAngle(2) : 0f;  // elbow
        float a4 = _robot != null ? GetJointAngle(3) : 0f;  // pitch

        float g  = 9.81f;

        // Convert to radians
        float s2 = a2 * Mathf.Deg2Rad;
        float s3 = a3 * Mathf.Deg2Rad;
        float s4 = a4 * Mathf.Deg2Rad;

        // Cumulative angle from vertical for each link
        // Link 1 (upper arm): angle = shoulder angle from vertical
        // Link 2 (forearm):   angle = shoulder + elbow
        // Link 3 (wrist):     angle = shoulder + elbow + pitch
        float angle1 = s2;
        float angle2 = s2 + s3;
        float angle3 = s2 + s3 + s4;

        // Horizontal reach of each link's centre of mass
        // (approximated as midpoint of link)
        float l0 = linkLength.Length > 0 ? linkLength[0] : 0.20f;
        float l1 = linkLength.Length > 1 ? linkLength[1] : 0.17f;
        float l2 = linkLength.Length > 2 ? linkLength[2] : 0.10f;
        float l3 = linkLength.Length > 3 ? linkLength[3] : 0.07f;

        float m0 = linkMass.Length > 0 ? linkMass[0] : 0.8f;
        float m1 = linkMass.Length > 1 ? linkMass[1] : 0.6f;
        float m2 = linkMass.Length > 2 ? linkMass[2] : 0.4f;
        float m3 = linkMass.Length > 3 ? linkMass[3] : 0.2f;
        float m4 = linkMass.Length > 4 ? linkMass[4] : 0.1f;

        // Reaches from J2 (shoulder) — used for torques at joints 1 and 2
        float reach1 = Mathf.Abs(Mathf.Sin(angle1) * (l0 * 0.5f));
        float reach2 = Mathf.Abs(Mathf.Sin(angle1) * l0 + Mathf.Sin(angle2) * (l1 * 0.5f));
        float reach3 = Mathf.Abs(Mathf.Sin(angle1) * l0 + Mathf.Sin(angle2) * l1
                                + Mathf.Sin(angle3) * (l2 * 0.5f));
        float reach4 = Mathf.Abs(Mathf.Sin(angle1) * l0 + Mathf.Sin(angle2) * l1
                                + Mathf.Sin(angle3) * l2 + Mathf.Sin(angle3) * (l3 * 0.5f));

        // Reaches from J3 (elbow) — used for torque at joint 3
        float reach2_J3 = Mathf.Abs(Mathf.Sin(angle2) * (l1 * 0.5f));
        float reach3_J3 = Mathf.Abs(Mathf.Sin(angle2) * l1 + Mathf.Sin(angle3) * (l2 * 0.5f));
        float reach4_J3 = Mathf.Abs(Mathf.Sin(angle2) * l1 + Mathf.Sin(angle3) * l2
                                   + Mathf.Sin(angle3) * (l3 * 0.5f));

        // Reaches from J4 (wrist pitch) — used for torque at joint 4
        float reach3_J4 = Mathf.Abs(Mathf.Sin(angle3) * (l2 * 0.5f));
        float reach4_J4 = Mathf.Abs(Mathf.Sin(angle3) * l2 + Mathf.Sin(angle3) * (l3 * 0.5f));

        // Torque at each joint = sum of (mass * g * reach_from_that_joint) for all distal links
        float torque1 = (m0 * g * reach1) + (m1 * g * reach2)
                      + (m2 * g * reach3) + (m3 * g * reach4);
        float torque2 = torque1;  // shoulder takes same load as base in this model
        float torque3 = (m1 * g * reach2_J3) + (m2 * g * reach3_J3) + (m3 * g * reach4_J3);
        float torque4 = (m2 * g * reach3_J4) + (m3 * g * reach4_J4);
        float torque5 = m4 * g * 0.03f;  // roll: minimal load, just end effector

        // Normalise against max rated torque
        float max0 = maxTorque.Length > 0 ? maxTorque[0] : 15f;
        float max1 = maxTorque.Length > 1 ? maxTorque[1] : 12f;
        float max2 = maxTorque.Length > 2 ? maxTorque[2] :  8f;
        float max3 = maxTorque.Length > 3 ? maxTorque[3] :  4f;
        float max4 = maxTorque.Length > 4 ? maxTorque[4] :  2f;

        _stressRatio[0] = Mathf.Clamp01(torque1 / max0);
        _stressRatio[1] = Mathf.Clamp01(torque2 / max1);
        _stressRatio[2] = Mathf.Clamp01(torque3 / max2);
        _stressRatio[3] = Mathf.Clamp01(torque4 / max3);
        _stressRatio[4] = Mathf.Clamp01(torque5 / max4);
    }

    // ─────────────────────────────────────────────
    // Visuals
    // ─────────────────────────────────────────────

    void ApplyHeatMap()
    {
        for (int i = 0; i < 5; i++)
        {
            if (_stressMaterials[i] == null) continue;

            Color c = StressColor(_stressRatio[i]);

            // Pulse animation for high-stress joints
            if (_stressRatio[i] >= pulseThreshold)
            {
                float pulse = (Mathf.Sin(Time.time * pulseSpeed) + 1f) * 0.5f;
                c = Color.Lerp(c, Color.white, pulse * 0.3f);
            }

            _stressMaterials[i].SetColor(ColorProp, c);
        }

        // Gripper inherits end-effector stress
        if (_stressMaterials[5] != null)
            _stressMaterials[5].SetColor(ColorProp, StressColor(_stressRatio[4]));
    }

    Color StressColor(float ratio)
    {
        if (ratio < 0.5f)
            return Color.Lerp(colorLow, colorMid, ratio * 2f);
        else
            return Color.Lerp(colorMid, colorHigh, (ratio - 0.5f) * 2f);
    }

    void UpdateStressLabels()
    {
        string[] names = { "Base", "Shoulder", "Elbow", "Pitch", "Roll" };
        for (int i = 0; i < 5; i++)
        {
            if (_labels[i] == null) continue;
            int pct = Mathf.RoundToInt(_stressRatio[i] * 100f);
            _labels[i].text     = $"{names[i]}\n{pct}%";
            _labels[i].color    = StressColor(_stressRatio[i]);
            _labels[i].gameObject.SetActive(true);
        }
    }

    void HideStressLabels()
    {
        foreach (var label in _labels)
            if (label != null) label.gameObject.SetActive(false);
    }

    void EnableStressMaterials()
    {
        for (int i = 0; i < _renderers.Length; i++)
            if (_renderers[i] != null && _stressMaterials[i] != null)
                _renderers[i].material = _stressMaterials[i];
    }

    void RestoreOriginalMaterials()
    {
        for (int i = 0; i < _renderers.Length; i++)
            if (_renderers[i] != null && _originalMaterials[i] != null)
                _renderers[i].material = _originalMaterials[i];
    }

    // ─────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────

    /// <summary>
    /// Reads the current smoothed angle from ER9ProFullController
    /// via reflection so we don't need to make private fields public.
    /// jointIndex: 0=a1(base), 1=a2(shoulder), 2=a3(elbow), 3=a4(pitch), 4=a5(roll)
    /// </summary>
    float GetJointAngle(int jointIndex)
    {
        if (_robot == null) return 0f;

        // Access private fields via reflection
        var field = typeof(ER9ProFullController).GetField(
            $"a{jointIndex + 1}",
            System.Reflection.BindingFlags.NonPublic |
            System.Reflection.BindingFlags.Instance
        );
        return field != null ? (float)field.GetValue(_robot) : 0f;
    }
}