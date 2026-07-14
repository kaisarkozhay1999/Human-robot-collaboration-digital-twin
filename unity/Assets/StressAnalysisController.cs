using TMPro;
using UnityEngine;

/// <summary>
/// Joint load visualization for the ER9 Pro robotic arm.
///
/// Computes a simplified static gravity load estimate from the current pose,
/// normalizes against rated joint torque limits, and maps the result to a
/// blue -> green -> red color gradient on the arm surfaces.
///
/// This is not finite-element stress analysis. It estimates static gravity
/// loading by joint axis, which is more defensible for an interactive heatmap.
/// </summary>
public class StressAnalysisController : MonoBehaviour
{
    [Header("Joint Renderers (base -> roll)")]
    public Renderer joint1Renderer;
    public Renderer joint2Renderer;
    public Renderer joint3Renderer;
    public Renderer joint4Renderer;
    public Renderer joint5Renderer;
    public Renderer gripperRenderer;

    [Header("UI")]
    public TMP_Text stressButtonLabel;
    public TMP_Text payloadLabel;
    public TMP_InputField payloadInput;

    [Header("Stress Labels")]
    public TMP_Text stressLabel1;
    public TMP_Text stressLabel2;
    public TMP_Text stressLabel3;
    public TMP_Text stressLabel4;
    public TMP_Text stressLabel5;

    [Header("Physical Parameters")]
    [Tooltip("Masses in kg: link1..link5")]
    public float[] linkMass = { 0.8f, 0.6f, 0.4f, 0.2f, 0.1f };

    [Tooltip("Lengths in meters: link1..link4")]
    public float[] linkLength = { 0.2f, 0.17f, 0.10f, 0.07f };

    [Tooltip("Rated max torque per joint in Nm. This defines what counts as 100% load.")]
    public float[] maxTorque = { 15f, 12f, 8f, 4f, 2f };

    [Header("Payload")]
    [Tooltip("Payload mass in kg. Set via Inspector or UI buttons.")]
    public float payloadMass = 0f;

    [Tooltip("Distance from the wrist pitch axis to the payload center of mass in meters.")]
    public float payloadLeverArm = 0.03f;

    [Tooltip("Approximate center-of-mass offset from the roll axis in meters. Keep this at 0 for a symmetric wrist/tool.")]
    public float rollLeverArm = 0f;

    [Header("Heatmap Colors")]
    public Color colorLow = new Color(0f, 0.35f, 1f);
    public Color colorMid = new Color(0f, 1f, 0.3f);
    public Color colorHigh = new Color(1f, 0.15f, 0f);

    [Header("Visual Settings")]
    [Range(0f, 1f)]
    [Tooltip("How strongly the heat color overrides the original material color. 0 = invisible, 1 = full replacement.")]
    public float overlayStrength = 0.70f;

    public float emissionStrength = 1.2f;
    public float pulseThreshold = 0.75f;
    public float pulseSpeed = 3f;

    bool _stressMode;
    readonly float[] _stressRatio = new float[5];
    Renderer[] _renderers;
    TMP_Text[] _labels;

    Material[][] _originalMaterialsAll;
    Material[][] _stressMaterialsAll;
    Color[][] _originalColors;
    bool[][] _shouldTint;

    ER9ProFullController _robot;

    static readonly int ColorProp = Shader.PropertyToID("_Color");
    static readonly int EmissionProp = Shader.PropertyToID("_EmissionColor");
    static readonly string[] MetallicKeywords = { "titanium", "silver", "steel" };
    static readonly string[] JointNames = { "Base", "Shoulder", "Elbow", "Pitch", "Roll" };

    void Awake()
    {
        _robot = GetComponent<ER9ProFullController>();
        ValidateConfiguration();

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

        int rendererCount = _renderers.Length;
        _originalMaterialsAll = new Material[rendererCount][];
        _stressMaterialsAll = new Material[rendererCount][];
        _originalColors = new Color[rendererCount][];
        _shouldTint = new bool[rendererCount][];

        for (int i = 0; i < rendererCount; i++)
        {
            if (_renderers[i] == null) continue;

            Material[] materials = _renderers[i].materials;
            int materialCount = materials.Length;

            _originalMaterialsAll[i] = materials;
            _stressMaterialsAll[i] = new Material[materialCount];
            _originalColors[i] = new Color[materialCount];
            _shouldTint[i] = new bool[materialCount];

            for (int j = 0; j < materialCount; j++)
            {
                Material sourceMaterial = materials[j];
                _stressMaterialsAll[i][j] = new Material(sourceMaterial);
                _shouldTint[i][j] = ShouldTint(sourceMaterial);
                _originalColors[i][j] = sourceMaterial.HasProperty(ColorProp)
                    ? sourceMaterial.color
                    : Color.white;
            }
        }

        if (payloadInput != null)
            payloadInput.onEndEdit.AddListener(OnPayloadInputChanged);

        HideStressLabels();
        UpdatePayloadLabel();
    }

    void OnValidate()
    {
        overlayStrength = Mathf.Clamp01(overlayStrength);
        payloadMass = Mathf.Max(0f, payloadMass);
        payloadLeverArm = Mathf.Max(0f, payloadLeverArm);
        rollLeverArm = Mathf.Max(0f, rollLeverArm);
        EnsureArrayLengths();
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
        if (payloadInput != null)
            payloadInput.onEndEdit.RemoveListener(OnPayloadInputChanged);

        if (_stressMaterialsAll == null) return;

        foreach (Material[] materials in _stressMaterialsAll)
        {
            if (materials == null) continue;
            foreach (Material material in materials)
            {
                if (material != null)
                    Destroy(material);
            }
        }
    }

    public void SetPayloadGrams(float grams)
    {
        payloadMass = Mathf.Max(0f, grams / 1000f);
        UpdatePayloadLabel();
        RefreshStressView();
    }

    public void SetPayloadKg(float kg)
    {
        payloadMass = Mathf.Max(0f, kg);
        UpdatePayloadLabel();
        RefreshStressView();
    }

    public void ClearPayload()
    {
        payloadMass = 0f;
        UpdatePayloadLabel();
        RefreshStressView();
    }

    public void ToggleStressAnalysis()
    {
        _stressMode = !_stressMode;

        if (stressButtonLabel != null)
            stressButtonLabel.text = _stressMode ? "Normal View" : "Stress View";

        if (_stressMode)
        {
            EnableStressMaterials();
            RefreshStressView();
        }
        else
        {
            RestoreOriginalMaterials();
            HideStressLabels();
        }
    }

    void OnPayloadInputChanged(string value)
    {
        if (float.TryParse(
            value,
            System.Globalization.NumberStyles.Float,
            System.Globalization.CultureInfo.InvariantCulture,
            out float grams))
        {
            SetPayloadGrams(grams);
        }
    }

    void UpdatePayloadLabel()
    {
        if (payloadLabel == null) return;

        payloadLabel.text = payloadMass <= 0f
            ? "No payload"
            : $"Payload: {payloadMass * 1000f:F0}g";
    }

    bool ShouldTint(Material material)
    {
        if (material == null || !material.HasProperty(ColorProp)) return false;

        string materialName = material.name.ToLowerInvariant();
        foreach (string keyword in MetallicKeywords)
        {
            if (materialName.Contains(keyword))
                return false;
        }

        return true;
    }

    void CalculateStress()
    {
        if (_robot == null)
        {
            ClearStressRatios();
            return;
        }

        float gravity = 9.81f;

        float shoulderAngle = (_robot.CurrentA2 + 90f) * Mathf.Deg2Rad;
        float elbowAngle = (_robot.CurrentA2 + _robot.CurrentA3 + 90f) * Mathf.Deg2Rad;
        float wristAngle = (_robot.CurrentA2 + _robot.CurrentA3 + _robot.CurrentA4 + 90f) * Mathf.Deg2Rad;

        float l0 = linkLength[0];
        float l1 = linkLength[1];
        float l2 = linkLength[2];
        float l3 = linkLength[3];

        float m0 = linkMass[0];
        float m1 = linkMass[1];
        float m2 = linkMass[2];
        float m3 = linkMass[3];
        float m4 = linkMass[4];
        float mp = payloadMass;

        float reachShoulderLink1 = Mathf.Abs(Mathf.Sin(shoulderAngle) * (l0 * 0.5f));
        float reachShoulderLink2 = Mathf.Abs(
            Mathf.Sin(shoulderAngle) * l0 +
            Mathf.Sin(elbowAngle) * (l1 * 0.5f));
        float reachShoulderLink3 = Mathf.Abs(
            Mathf.Sin(shoulderAngle) * l0 +
            Mathf.Sin(elbowAngle) * l1 +
            Mathf.Sin(wristAngle) * (l2 * 0.5f));
        float reachShoulderLink4 = Mathf.Abs(
            Mathf.Sin(shoulderAngle) * l0 +
            Mathf.Sin(elbowAngle) * l1 +
            Mathf.Sin(wristAngle) * (l2 + l3 * 0.5f));
        float reachShoulderPayload = Mathf.Abs(
            Mathf.Sin(shoulderAngle) * l0 +
            Mathf.Sin(elbowAngle) * l1 +
            Mathf.Sin(wristAngle) * (l2 + l3 + payloadLeverArm));

        float reachElbowLink2 = Mathf.Abs(Mathf.Sin(elbowAngle) * (l1 * 0.5f));
        float reachElbowLink3 = Mathf.Abs(
            Mathf.Sin(elbowAngle) * l1 +
            Mathf.Sin(wristAngle) * (l2 * 0.5f));
        float reachElbowLink4 = Mathf.Abs(
            Mathf.Sin(elbowAngle) * l1 +
            Mathf.Sin(wristAngle) * (l2 + l3 * 0.5f));
        float reachElbowPayload = Mathf.Abs(
            Mathf.Sin(elbowAngle) * l1 +
            Mathf.Sin(wristAngle) * (l2 + l3 + payloadLeverArm));

        float wristMassAtCom = m3 * (l3 * 0.5f) + m4 * l3 + mp * (l3 + payloadLeverArm);
        float pitchTorque = gravity * Mathf.Abs(Mathf.Sin(wristAngle)) * wristMassAtCom;

        float baseTorque = 0f;
        float shoulderTorque =
            m0 * gravity * reachShoulderLink1 +
            m1 * gravity * reachShoulderLink2 +
            m2 * gravity * reachShoulderLink3 +
            m3 * gravity * reachShoulderLink4 +
            m4 * gravity * reachShoulderLink4 +
            mp * gravity * reachShoulderPayload;
        float elbowTorque =
            m1 * gravity * reachElbowLink2 +
            m2 * gravity * reachElbowLink3 +
            m3 * gravity * reachElbowLink4 +
            m4 * gravity * reachElbowLink4 +
            mp * gravity * reachElbowPayload;
        float rollTorque = (m4 + mp) * gravity * rollLeverArm;

        _stressRatio[0] = SafeNormalize(baseTorque, maxTorque[0], JointNames[0]);
        _stressRatio[1] = SafeNormalize(shoulderTorque, maxTorque[1], JointNames[1]);
        _stressRatio[2] = SafeNormalize(elbowTorque, maxTorque[2], JointNames[2]);
        _stressRatio[3] = SafeNormalize(pitchTorque, maxTorque[3], JointNames[3]);
        _stressRatio[4] = SafeNormalize(rollTorque, maxTorque[4], JointNames[4]);
    }

    void ApplyHeatMap()
    {
        for (int i = 0; i < _renderers.Length; i++)
        {
            if (_renderers[i] == null || _stressMaterialsAll[i] == null) continue;

            int ratioIndex = Mathf.Min(i, _stressRatio.Length - 1);
            float ratio = _stressRatio[ratioIndex];
            Color stressColor = StressColor(ratio);

            if (ratio >= pulseThreshold)
            {
                float pulse = (Mathf.Sin(Time.time * pulseSpeed) + 1f) * 0.5f;
                stressColor = Color.Lerp(stressColor, Color.white, pulse * 0.25f);
            }

            for (int j = 0; j < _stressMaterialsAll[i].Length; j++)
            {
                Material material = _stressMaterialsAll[i][j];
                if (material == null || !_shouldTint[i][j]) continue;

                Color finalColor = Color.Lerp(
                    _originalColors[i][j],
                    stressColor,
                    overlayStrength);

                material.SetColor(ColorProp, finalColor);
                material.EnableKeyword("_EMISSION");
                material.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;
                material.SetColor(EmissionProp, stressColor * ratio * emissionStrength);
            }
        }
    }

    Color StressColor(float ratio)
    {
        return ratio < 0.5f
            ? Color.Lerp(colorLow, colorMid, ratio * 2f)
            : Color.Lerp(colorMid, colorHigh, (ratio - 0.5f) * 2f);
    }

    void UpdateStressLabels()
    {
        for (int i = 0; i < _labels.Length; i++)
        {
            if (_labels[i] == null) continue;

            int percent = Mathf.RoundToInt(_stressRatio[i] * 100f);
            _labels[i].text = $"{JointNames[i]}\n{percent}%";
            _labels[i].color = StressColor(_stressRatio[i]);
            _labels[i].gameObject.SetActive(true);
        }
    }

    void HideStressLabels()
    {
        if (_labels == null) return;

        foreach (TMP_Text label in _labels)
        {
            if (label != null)
                label.gameObject.SetActive(false);
        }
    }

    void EnableStressMaterials()
    {
        for (int i = 0; i < _renderers.Length; i++)
        {
            if (_renderers[i] != null && _stressMaterialsAll[i] != null)
                _renderers[i].materials = _stressMaterialsAll[i];
        }
    }

    void RestoreOriginalMaterials()
    {
        for (int i = 0; i < _renderers.Length; i++)
        {
            if (_renderers[i] != null && _originalMaterialsAll[i] != null)
                _renderers[i].materials = _originalMaterialsAll[i];
        }
    }

    void RefreshStressView()
    {
        if (!_stressMode) return;

        CalculateStress();
        ApplyHeatMap();
        UpdateStressLabels();
    }

    void ValidateConfiguration()
    {
        EnsureArrayLengths();

        if (_robot == null)
            Debug.LogWarning("[Stress] ER9ProFullController is required on the same GameObject.");
    }

    void EnsureArrayLengths()
    {
        EnsureArraySize(ref linkMass, 5, new float[] { 0.8f, 0.6f, 0.4f, 0.2f, 0.1f });
        EnsureArraySize(ref linkLength, 4, new float[] { 0.2f, 0.17f, 0.10f, 0.07f });
        EnsureArraySize(ref maxTorque, 5, new float[] { 15f, 12f, 8f, 4f, 2f });
    }

    void EnsureArraySize(ref float[] values, int expectedLength, float[] fallback)
    {
        if (values != null && values.Length == expectedLength) return;

        float[] resized = new float[expectedLength];
        for (int i = 0; i < expectedLength; i++)
        {
            bool useExistingValue = values != null && i < values.Length;
            resized[i] = useExistingValue ? values[i] : fallback[i];
        }

        values = resized;
    }

    void ClearStressRatios()
    {
        for (int i = 0; i < _stressRatio.Length; i++)
            _stressRatio[i] = 0f;
    }

    float SafeNormalize(float torque, float ratedTorque, string jointName)
    {
        if (ratedTorque <= 0f)
        {
            Debug.LogWarning($"[Stress] Rated torque for {jointName} must be greater than zero.");
            return 0f;
        }

        return Mathf.Clamp01(torque / ratedTorque);
    }
}
