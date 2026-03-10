using UnityEngine;
using TMPro; // remove this line if you're not using TMP

public class ZoneUIReceiver : MonoBehaviour
{
    [Header("Assign one of these")]
    public TMP_Text tmpText;     // drag your TMP text here
    public UnityEngine.UI.Text uiText; // or drag legacy Text here

    public void SetZone(string zone)
    {
        zone = (zone ?? "").Trim().ToUpperInvariant();

        if (tmpText != null) tmpText.text = $"ZONE: {zone}";
        if (uiText != null) uiText.text = $"ZONE: {zone}";
    }
}