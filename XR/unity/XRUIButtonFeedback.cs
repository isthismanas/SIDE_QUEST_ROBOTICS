using System.Collections;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class XRUIButtonFeedback : MonoBehaviour,
    IPointerDownHandler,
    IPointerUpHandler
{
    [Header("References")]
    [SerializeField] private RectTransform target;
    [SerializeField] private Button button;
    [SerializeField] private UIAudioPlayer audioPlayer;
    [SerializeField] private UIHapticPlayer hapticPlayer;

    [Header("Audio Type")]
    [SerializeField] private bool usePrimaryClick = false;

    [Header("Motion")]
    [SerializeField] private float pressedScale = 0.97f;
    [SerializeField] private float scaleSpeed = 20f;
    [SerializeField] private float autoReleaseDelay = 0.08f;

    private Vector3 _baseScale;
    private Vector3 _targetScale;
    private bool _pressed;
    private Coroutine _releaseRoutine;

    private void Reset()
    {
        target = GetComponent<RectTransform>();
        button = GetComponent<Button>();
    }

    private void Awake()
    {
        if (target == null)
            target = GetComponent<RectTransform>();

        if (button == null)
            button = GetComponent<Button>();

        _baseScale = target.localScale;
        _targetScale = _baseScale;

        if (button != null)
            button.onClick.AddListener(HandleClick);
    }

    private void OnEnable()
    {
        if (target == null)
            target = GetComponent<RectTransform>();

        _pressed = false;
        _baseScale = target.localScale;
        _targetScale = _baseScale;
        target.localScale = _baseScale;
    }

    private void OnDisable()
    {
        _pressed = false;

        if (target != null)
        {
            target.localScale = _baseScale;
            _targetScale = _baseScale;
        }

        if (_releaseRoutine != null)
        {
            StopCoroutine(_releaseRoutine);
            _releaseRoutine = null;
        }
    }

    private void OnDestroy()
    {
        if (button != null)
            button.onClick.RemoveListener(HandleClick);
    }

    private void Update()
    {
        if (target == null) return;

        target.localScale = Vector3.Lerp(
            target.localScale,
            _targetScale,
            Time.unscaledDeltaTime * scaleSpeed
        );
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        _pressed = true;
        UpdateTargetScale();

        if (_releaseRoutine != null)
            StopCoroutine(_releaseRoutine);
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        ReleasePress();
    }

    private void HandleClick()
    {
        if (audioPlayer != null)
        {
            if (usePrimaryClick)
                audioPlayer.PlayPrimary();
            else
                audioPlayer.PlaySecondary();
        }

        if (hapticPlayer != null)
            hapticPlayer.BuzzClick();

        if (_releaseRoutine != null)
            StopCoroutine(_releaseRoutine);

        _releaseRoutine = StartCoroutine(AutoRelease());
    }

    private IEnumerator AutoRelease()
    {
        yield return new WaitForSecondsRealtime(autoReleaseDelay);
        ReleasePress();
        _releaseRoutine = null;
    }

    private void ReleasePress()
    {
        _pressed = false;
        UpdateTargetScale();
    }

    private void UpdateTargetScale()
    {
        _targetScale = _pressed
            ? _baseScale * pressedScale
            : _baseScale;
    }
}