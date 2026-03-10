using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit.Inputs.Haptics;

public class UIHapticPlayer : MonoBehaviour
{
    [Header("Controller Outputs")]
    [SerializeField] private HapticImpulsePlayer leftHaptics;
    [SerializeField] private HapticImpulsePlayer rightHaptics;

    [Header("Hover")]
    [SerializeField] private float hoverAmplitude = 0.2f;
    [SerializeField] private float hoverDuration = 0.03f;

    [Header("Click")]
    [SerializeField] private float clickAmplitude = 0.55f;
    [SerializeField] private float clickDuration = 0.06f;

    public void BuzzHover()
    {
        SendBoth(hoverAmplitude, hoverDuration);
    }

    public void BuzzClick()
    {
        SendBoth(clickAmplitude, clickDuration);
    }

    private void SendBoth(float amplitude, float duration)
    {
        if (leftHaptics != null)
            leftHaptics.SendHapticImpulse(amplitude, duration);

        if (rightHaptics != null)
            rightHaptics.SendHapticImpulse(amplitude, duration);
    }
}