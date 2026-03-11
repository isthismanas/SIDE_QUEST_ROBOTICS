using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class BoostUIController : MonoBehaviour
{
    [Header("Boost Indicators")]
    public Image circle1;
    public Image circle2;
    public Image circle3;
    public Image frame;

    [Header("Colors")]
    public Color inactiveColor = new Color(0.3f, 0.3f, 0.3f, 0.7f);
    public Color activeCircleColor = new Color(0.0f, 0.95f, 0.45f, 1.0f);
    public Color activeFrameColor = new Color(0.0f, 1.0f, 0.6f, 1.0f);

    [Header("Pulse")]
    [Min(0.1f)] public float pulseSpeed = 6.0f;
    public Color pulseColorA = new Color(0.0f, 1.0f, 0.0f, 1.0f);
    public Color pulseColorB = new Color(1.0f, 0.9f, 0.0f, 1.0f);

    [Header("Pop Animation")]
    [Min(0.03f)] public float popDuration = 0.18f;
    [Min(1.0f)] public float popStartScale = 1.1f;

    private Coroutine _framePulseRoutine;
    private Coroutine _framePopRoutine;
    private bool _boostLatched;

    public void SetBoostState(int comboCount, bool boostActive)
    {
        int combo = Mathf.Clamp(comboCount, 0, 3);
        bool wasLatched = _boostLatched;
        if (boostActive)
            _boostLatched = true;

        if (_boostLatched)
        {
            SetIndicator(circle1, true, activeCircleColor);
            SetIndicator(circle2, true, activeCircleColor);
            SetIndicator(circle3, true, activeCircleColor);
            if (frame != null)
            {
                frame.enabled = true;
                if (_framePulseRoutine == null)
                    frame.color = pulseColorA;
            }
            if (!wasLatched)
                PlayFramePopOnce();
            EnsurePulseRunning();
            return;
        }

        bool c1On = combo >= 1;
        bool c2On = combo >= 2;
        bool c3On = combo >= 3;

        SetIndicator(circle1, c1On, activeCircleColor);
        SetIndicator(circle2, c2On, activeCircleColor);
        SetIndicator(circle3, c3On, activeCircleColor);

        HideFrameAndReset();
        StopPulse();
    }

    public void OnBoostEnded()
    {
        _boostLatched = false;
        SetIndicator(circle1, false, activeCircleColor);
        SetIndicator(circle2, false, activeCircleColor);
        SetIndicator(circle3, false, activeCircleColor);
        HideFrameAndReset();
        StopPulse();
    }

    private void SetIndicator(Image target, bool active, Color activeColor)
    {
        if (target == null)
            return;

        target.color = active ? activeColor : inactiveColor;
    }

    private void EnsurePulseRunning()
    {
        if (frame == null)
            return;

        if (_framePulseRoutine != null)
            return;

        _framePulseRoutine = StartCoroutine(FramePulseLoop());
        Debug.Log("[BOOST_UI] pulse started");
    }

    private void StopPulse()
    {
        if (_framePulseRoutine != null)
        {
            StopCoroutine(_framePulseRoutine);
            _framePulseRoutine = null;
            Debug.Log("[BOOST_UI] pulse stopped");
        }
    }

    private void HideFrameAndReset()
    {
        if (_framePopRoutine != null)
        {
            StopCoroutine(_framePopRoutine);
            _framePopRoutine = null;
        }

        if (frame == null)
            return;

        frame.enabled = false;
        frame.rectTransform.localScale = Vector3.one;
    }

    private void PlayFramePopOnce()
    {
        if (frame == null)
            return;

        if (_framePopRoutine != null)
            StopCoroutine(_framePopRoutine);

        _framePopRoutine = StartCoroutine(FramePopRoutine());
    }

    private IEnumerator FramePopRoutine()
    {
        float duration = Mathf.Max(0.03f, popDuration);
        Vector3 start = Vector3.one * Mathf.Max(1.0f, popStartScale);
        Vector3 end = Vector3.one;
        frame.rectTransform.localScale = start;

        float t = 0f;
        while (t < duration)
        {
            t += Time.unscaledDeltaTime;
            float p = Mathf.Clamp01(t / duration);
            frame.rectTransform.localScale = Vector3.Lerp(start, end, p);
            yield return null;
        }

        frame.rectTransform.localScale = end;
        _framePopRoutine = null;
    }

    private IEnumerator FramePulseLoop()
    {
        while (true)
        {
            if (frame == null)
            {
                _framePulseRoutine = null;
                yield break;
            }

            float t = (Mathf.Sin(Time.unscaledTime * pulseSpeed) + 1.0f) * 0.5f;
            Color a = pulseColorA;
            Color b = pulseColorB;
            a.a = 1.0f;
            b.a = 1.0f;
            frame.color = Color.Lerp(a, b, t);
            yield return null;
        }
    }

    private void OnDisable()
    {
        _boostLatched = false;
        HideFrameAndReset();
        StopPulse();
    }
}
