using UnityEngine;

public class ZoneIndicator : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Drag the GameObject that has RobotCommandPipe on it.")]
    public RobotCommandPipe pipe;

    [Tooltip("Renderer to tint (sphere mesh renderer). If null, auto-finds on this object/children.")]
    public Renderer targetRenderer;

    [Header("Colors")]
    public Color green = Color.green;
    public Color yellow = Color.yellow;
    public Color red = Color.red;

    [Header("Shader Color Property")]
    [Tooltip("URP/Lit usually uses _BaseColor. Built-in Standard uses _Color.")]
    public string primaryColorProperty = "_BaseColor";
    public string fallbackColorProperty = "_Color";

    [Header("Debug")]
    public bool log;

    private MaterialPropertyBlock _mpb;
    private int _propPrimary;
    private int _propFallback;

    private void Awake()
    {
        if (targetRenderer == null)
            targetRenderer = GetComponentInChildren<Renderer>();

        _mpb = new MaterialPropertyBlock();
        _propPrimary = Shader.PropertyToID(primaryColorProperty);
        _propFallback = Shader.PropertyToID(fallbackColorProperty);
    }

    private void OnEnable()
    {
        if (pipe != null && pipe.OnZoneChanged != null)
        {
            pipe.OnZoneChanged.AddListener(HandleZoneChanged);

            // Apply current value immediately (so indicator isn't stale at start)
            HandleZoneChanged(pipe.currentZone);
        }
        else
        {
            if (log) Debug.LogWarning("[ZoneIndicator] Pipe not assigned (or OnZoneChanged missing).");
        }
    }

    private void OnDisable()
    {
        if (pipe != null && pipe.OnZoneChanged != null)
        {
            pipe.OnZoneChanged.RemoveListener(HandleZoneChanged);
        }
    }

    private void HandleZoneChanged(string zone)
    {
        if (targetRenderer == null)
        {
            if (log) Debug.LogWarning("[ZoneIndicator] targetRenderer is null.");
            return;
        }

        string z = (zone ?? "").Trim().ToUpperInvariant();

        Color c = green; // default
        if (z == "YELLOW") c = yellow;
        else if (z == "RED") c = red;
        else if (z == "GREEN") c = green;

        ApplyColor(c);

        if (log) Debug.Log($"[ZoneIndicator] zone='{z}' -> color={c}");
    }

    private void ApplyColor(Color color)
    {
        // Use MPB so we don't instantiate materials at runtime
        targetRenderer.GetPropertyBlock(_mpb);

        // Prefer primary property if the material supports it, else fallback
        var mat = targetRenderer.sharedMaterial;
        if (mat != null && mat.HasProperty(_propPrimary))
        {
            _mpb.SetColor(_propPrimary, color);
        }
        else
        {
            _mpb.SetColor(_propFallback, color);
        }

        targetRenderer.SetPropertyBlock(_mpb);
    }
}