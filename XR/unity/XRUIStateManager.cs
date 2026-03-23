using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections;

public class XRUIStateManager : MonoBehaviour
{
    public enum UIState { Boot, Starting, Running, FixMode }

    [Header("Config")]
    [Tooltip("If true, dev-only buttons become visible in all states.")]
    public bool DEV_MODE = false;

    [Header("Core Buttons (assign GameObjects)")]
    public GameObject Button_Start;

    public GameObject Button_TopView;     // Camera Inspector
    public GameObject Button_SideView;    // Camera Manager

    public GameObject Button_Drop;
    public GameObject Button_Fix;

    public GameObject Button_NudgeLeft;
    public GameObject Button_NudgeRight;
    public GameObject Button_NudgeForward;
    public GameObject Button_NudgeBackward;

    [Header("Dev Buttons (assign GameObjects)")]
    public GameObject Button_Home;
    public GameObject Button_Cancel;
    public GameObject Button_GripperToggle;
    public GameObject Button_GripperOpen;
    public GameObject Button_GripperClose;

    [Header("Debug")]
    [SerializeField] private UIState _state = UIState.Boot;

    [Header("Start ACK")]
    [Tooltip("How long to wait for ACK START before restoring Boot state.")]
    public float startAckTimeoutSeconds = 5f;

    [Header("Fix ACK")]
    [Tooltip("How long to wait for ACK/NACK FIX before clearing pending state.")]
    public float FixAckTimeoutSeconds = 2.0f;

    [Header("Start Countdown")]
    public RobotCommandPipe commandPipe;
    [Min(1)] public int countdownSeconds = 3;
    [Tooltip("Status/readiness text owner (Waiting/Ready/Run/Game Over).")]
    public TMP_Text statusLabel;
    [Tooltip("Optional countdown label. If unassigned, countdown runs without text.")]
    public TMP_Text countdownLabel;
    [Tooltip("Optional live elapsed run timer label.")]
    public TMP_Text timerLabel;

    [Header("Run End Message")]
    [Tooltip("How long to keep GAME OVER visible before resetting to boot.")]
    [Min(0f)] public float runEndMessageSeconds = 7f;

    [Header("Audio")]
    [Tooltip("Optional music ducking controller. Lowered only after START is acknowledged.")]
    public MusicDucker musicDucker;
    [Tooltip("Optional UI audio player for gameplay event SFX.")]
    public UIAudioPlayer audioPlayer;

    [Header("Boost UI")]
    [Tooltip("Optional Boost UI controller for BOOST_STATE updates from RobotCommandPipe.")]
    public BoostUIController boostUIController;

    private bool _waitingForStartAck = false;
    private float _startAckDeadline = 0f;
    private Coroutine _startCountdownRoutine = null;
    private bool _waitingForFixAck = false;
    private float _fixPendingStartTime = 0f;
    private bool _decisionWindowActive = false;
    private bool hasName = false;
    private bool _boostActive = false;
    private bool _runTimerActive = false;
    private float _runTimerStartUnscaled = 0f;
    private Coroutine _runEndRoutine = null;

    void Awake()
    {
        // Boot state on startup
        ApplyState(UIState.Boot);
        UpdateCountdownLabel("");
        ResetTimerDisplay();
        UpdateNameReadinessLabel();
    }

    void Update()
    {
        if (_waitingForStartAck && Time.unscaledTime >= _startAckDeadline)
        {
            OnStartNack("TIMEOUT");
        }

        if (_runTimerActive)
        {
            float elapsed = Mathf.Max(0f, Time.unscaledTime - _runTimerStartUnscaled);
            UpdateTimerLabel(FormatElapsedTime(elapsed));
        }
    }

    // ----- Public UI callbacks (wire these in Button OnClick) -----

    public void OnStartClicked()
    {
        if (_waitingForStartAck)
            return;

        if (!hasName)
        {
            UpdateNameReadinessLabel();
            ApplyState(UIState.Boot);
            return;
        }

        _waitingForStartAck = true;
        _startAckDeadline = Time.unscaledTime + Mathf.Max(0.1f, startAckTimeoutSeconds);
        ApplyState(UIState.Starting);
        UpdateStatusLabel("Starting...");

        if (_startCountdownRoutine != null)
            StopCoroutine(_startCountdownRoutine);
        _startCountdownRoutine = StartCoroutine(StartCountdownThenSend());
    }

    public void OnStartAck()
    {
        _waitingForStartAck = false;
        if (_startCountdownRoutine != null)
        {
            StopCoroutine(_startCountdownRoutine);
            _startCountdownRoutine = null;
        }
        UpdateCountdownLabel("");
        _decisionWindowActive = false;
        UpdateStatusLabel("Run started");
        StartRunTimer();
        musicDucker?.LowerMusic();
        ApplyState(UIState.Running);
    }

    public void OnStartNack(string reason)
    {
        _waitingForStartAck = false;
        _decisionWindowActive = false;
        _startAckDeadline = 0f;
        if (_startCountdownRoutine != null)
        {
            StopCoroutine(_startCountdownRoutine);
            _startCountdownRoutine = null;
        }
        Debug.LogWarning("[XRUIStateManager] START rejected: " + reason);
        ApplyState(UIState.Boot);
        UpdateNameReadinessLabel();
    }

    public void OnNameSet()
    {
        hasName = true;
        if (!_waitingForStartAck)
            UpdateNameReadinessLabel();
    }

    public void OnDecisionReady(int seq)
    {
        _decisionWindowActive = true;
        ApplyState(UIState.Running);
        var fixButton = Button_Fix != null ? Button_Fix.GetComponent<Button>() : null;
        var dropButton = Button_Drop != null ? Button_Drop.GetComponent<Button>() : null;
        var nudgeLeft = Button_NudgeLeft != null ? Button_NudgeLeft.GetComponent<Button>() : null;
        bool fixEnabled = fixButton != null && fixButton.interactable;
        bool dropEnabled = dropButton != null && dropButton.interactable;
        bool nudgeEnabled = nudgeLeft != null && nudgeLeft.interactable;
        Debug.Log($"[UI] Decision window opened. fix={fixEnabled} drop={dropEnabled} nudge={nudgeEnabled}");
    }

    public void OnBoostState(int comboCount, bool boostActive)
    {
        if (!_boostActive && boostActive)
        {
            audioPlayer?.PlayComboBoost();
            musicDucker?.BoostSwell();
        }
        if (boostActive) _boostActive = true;

        if (boostUIController == null)
            return;

        boostUIController.SetBoostState(comboCount, boostActive);
    }

    public void OnBoostEnded()
    {
        _boostActive = false;
        audioPlayer?.PlayBoostEnd();
        musicDucker?.LowerMusic();

        if (boostUIController == null)
            return;

        boostUIController.OnBoostEnded();
    }

    public void OnRunComplete()
    {
        _boostActive = false;
        audioPlayer?.PlayRunSuccess();
        musicDucker?.RestoreMusic();
        BeginRunEndFlow();
    }

    public void OnRunTumble()
    {
        _boostActive = false;
        audioPlayer?.PlayTumble();
        musicDucker?.RestoreMusic();
        BeginRunEndFlow();
    }

    public void OnGameplayComplete()
    {
        StopRunTimer();
    }

    public void ResetToBoot()
    {
        if (_runEndRoutine != null)
        {
            StopCoroutine(_runEndRoutine);
            _runEndRoutine = null;
        }
        hasName = false;
        _boostActive = false;
        _waitingForStartAck = false;
        _waitingForFixAck = false;
        _fixPendingStartTime = 0f;
        _decisionWindowActive = false;
        _startAckDeadline = 0f;
        StopRunTimer();
        ResetTimerDisplay();
        if (_startCountdownRoutine != null)
        {
            StopCoroutine(_startCountdownRoutine);
            _startCountdownRoutine = null;
        }
        UpdateCountdownLabel("");
        ApplyState(UIState.Boot);
        UpdateNameReadinessLabel();
    }

    public void OnDropClicked()
    {
        // Spec: no UI change
        // Keep whatever state we are currently in.
        ApplyState(_state);
    }

    public void OnDropAck()
    {
        _decisionWindowActive = false;
        string zone = commandPipe != null ? commandPipe.currentZone : null;
        audioPlayer?.PlayDropDelayed(zone);
        ExitFixMode();
    }

    public void OnDropNack(string reason)
    {
        Debug.LogWarning("[XRUIStateManager] DROP rejected: " + reason);
    }

    public void OnFixClicked()
    {
        Debug.Log($"[FIX][UI] click state={_state} pending={_waitingForFixAck} pipe={(commandPipe != null)}");
        if (_waitingForFixAck)
        {
            Debug.LogWarning("[FIX][UI] ignore: PENDING");
            return;
        }

        if (_state != UIState.Running)
        {
            Debug.LogWarning("[FIX][UI] ignore: WRONG_STATE");
            return;
        }

        if (commandPipe == null)
        {
            Debug.LogWarning("[FIX][UI] ignore: NO_PIPE");
            OnFixNack("NO_PIPE");
            return;
        }

        if (!commandPipe.HasDecisionToken())
        {
            Debug.LogWarning("[FIX][UI] ignore: NO_DECISION_TOKEN");
            return;
        }

        _waitingForFixAck = true;
        _fixPendingStartTime = Time.time;
        Debug.Log("[FIX][UI] pending=true; sending FIX");
        UpdateButtonInteractivity();
        commandPipe.SendFix();
    }

    public void OnFixAck()
    {
        Debug.Log($"[FIX][UI] ACK received; pending was={_waitingForFixAck} -> clearing and entering FixMode");
        _waitingForFixAck = false;
        _fixPendingStartTime = 0f;
        EnterFixMode();
    }

    public void OnFixNack(string reason)
    {
        Debug.LogWarning($"[FIX][UI] NACK received reason={reason}; pending was={_waitingForFixAck} -> clearing");
        _waitingForFixAck = false;
        _fixPendingStartTime = 0f;
        _decisionWindowActive = false;
        ExitFixMode();
    }

    public void OnNudgeLeftClicked()
    {
        // Spec decision: stay in FixMode (no auto-exit)
        ApplyState(_state);
    }

    public void OnNudgeRightClicked()
    {
        // Spec decision: stay in FixMode (no auto-exit)
        ApplyState(_state);
    }

    public void OnNudgeForwardClicked()
    {
        commandPipe?.NudgeUp();
        ApplyState(_state);
    }

    public void OnNudgeBackwardClicked()
    {
        commandPipe?.NudgeDown();
        ApplyState(_state);
    }

    // Optional future hook (for a Done button or Python signal)
    public void ExitFixMode()
    {
        ApplyState(UIState.Running);
        _decisionWindowActive = false;
        UpdateButtonInteractivity();
    }

    private void EnterFixMode()
    {
        ApplyState(UIState.FixMode);
        var fixButton = Button_Fix != null ? Button_Fix.GetComponent<Button>() : null;
        var dropButton = Button_Drop != null ? Button_Drop.GetComponent<Button>() : null;
        var nudgeLeft = Button_NudgeLeft != null ? Button_NudgeLeft.GetComponent<Button>() : null;
        bool fixEnabled = fixButton != null && fixButton.interactable;
        bool dropEnabled = dropButton != null && dropButton.interactable;
        bool nudgeEnabled = nudgeLeft != null && nudgeLeft.interactable;
        Debug.Log($"[UI] Entering FixMode. fix={fixEnabled} drop={dropEnabled} nudge={nudgeEnabled}");
    }

    private IEnumerator StartCountdownThenSend()
    {
        int seconds = Mathf.Max(1, countdownSeconds);
        for (int value = seconds; value >= 1; value--)
        {
            if (!_waitingForStartAck)
                yield break;

            UpdateCountdownLabel(value.ToString());
            yield return new WaitForSecondsRealtime(1f);
        }

        UpdateCountdownLabel("");

        if (!_waitingForStartAck)
            yield break;

        if (commandPipe == null)
        {
            OnStartNack("NO_PIPE");
            yield break;
        }

        commandPipe.SendStart();
        _startCountdownRoutine = null;
    }

    private void BeginRunEndFlow()
    {
        StopRunTimer();
        UpdateStatusLabel("GAME OVER");

        if (_runEndRoutine != null)
            StopCoroutine(_runEndRoutine);

        _runEndRoutine = StartCoroutine(ShowRunEndThenReset());
    }

    private IEnumerator ShowRunEndThenReset()
    {
        float delay = Mathf.Max(0f, runEndMessageSeconds);
        if (delay > 0f)
            yield return new WaitForSecondsRealtime(delay);

        _runEndRoutine = null;
        ResetToBoot();
    }

    // ----- Core state application -----

    private void ApplyState(UIState newState)
    {
        _state = newState;

        // Boot defaults
        SetActiveSafe(Button_Start, false);

        SetActiveSafe(Button_TopView, false);
        SetActiveSafe(Button_SideView, false);

        SetActiveSafe(Button_Drop, false);
        SetActiveSafe(Button_Fix, false);

        SetActiveSafe(Button_NudgeLeft, false);
        SetActiveSafe(Button_NudgeRight, false);
        SetActiveSafe(Button_NudgeForward, false);
        SetActiveSafe(Button_NudgeBackward, false);

        // State-specific enables
        switch (_state)
        {
            case UIState.Boot:
                SetActiveSafe(Button_Start, true);
                SetInteractableSafe(Button_Start, true);
                break;

            case UIState.Starting:
                SetActiveSafe(Button_Start, true);
                SetInteractableSafe(Button_Start, false);
                break;

            case UIState.Running:
                SetActiveSafe(Button_TopView, true);
                SetActiveSafe(Button_SideView, true);
                SetActiveSafe(Button_Drop, true);
                SetActiveSafe(Button_Fix, true);
                break;

            case UIState.FixMode:
                SetActiveSafe(Button_TopView, true);
                SetActiveSafe(Button_SideView, true);
                SetActiveSafe(Button_Drop, true);

                // Fix disappears, nudges appear
                SetActiveSafe(Button_Fix, false);
                SetActiveSafe(Button_NudgeLeft, true);
                SetActiveSafe(Button_NudgeRight, true);
                SetActiveSafe(Button_NudgeForward, true);
                SetActiveSafe(Button_NudgeBackward, true);
                break;
        }

        UpdateButtonInteractivity();
        ApplyDevVisibility();
    }

    private void ApplyDevVisibility()
    {
        bool show = DEV_MODE;

        SetActiveSafe(Button_Home, show);
        SetActiveSafe(Button_Cancel, show);
        SetActiveSafe(Button_GripperToggle, show);
        SetActiveSafe(Button_GripperOpen, show);
        SetActiveSafe(Button_GripperClose, show);
    }

    private static void SetActiveSafe(GameObject go, bool active)
    {
        if (go == null) return;
        if (go.activeSelf == active) return;
        go.SetActive(active);
    }

    private static void SetInteractableSafe(GameObject go, bool interactable)
    {
        if (go == null) return;
        var button = go.GetComponent<Button>();
        if (button == null) return;
        if (button.interactable == interactable) return;
        button.interactable = interactable;
    }

    private void UpdateButtonInteractivity()
    {
        bool fixEnabled = false;
        bool dropEnabled = false;
        bool nudgeEnabled = false;

        if (_state == UIState.FixMode)
        {
            fixEnabled = false;
            dropEnabled = true;
            nudgeEnabled = true;
        }
        else if (_state == UIState.Running && _decisionWindowActive)
        {
            fixEnabled = true;
            dropEnabled = true;
            nudgeEnabled = false;
        }

        if (_waitingForFixAck)
        {
            fixEnabled = false;
        }

        SetInteractableSafe(Button_Fix, fixEnabled);
        SetInteractableSafe(Button_Drop, dropEnabled);
        SetInteractableSafe(Button_NudgeLeft, nudgeEnabled);
        SetInteractableSafe(Button_NudgeRight, nudgeEnabled);
        SetInteractableSafe(Button_NudgeForward, nudgeEnabled);
        SetInteractableSafe(Button_NudgeBackward, nudgeEnabled);
    }

    private void UpdateStatusLabel(string text)
    {
        if (statusLabel == null)
            return;
        statusLabel.text = text ?? "";
    }

    private void UpdateCountdownLabel(string text)
    {
        if (countdownLabel == null)
            return;
        countdownLabel.text = text ?? "";
    }

    private void UpdateTimerLabel(string text)
    {
        if (timerLabel == null)
            return;
        timerLabel.text = text ?? "";
    }

    private void StartRunTimer()
    {
        _runTimerStartUnscaled = Time.unscaledTime;
        _runTimerActive = true;
        UpdateTimerLabel("00:00");
    }

    private void StopRunTimer()
    {
        _runTimerActive = false;
    }

    private void ResetTimerDisplay()
    {
        UpdateTimerLabel("00:00");
    }

    private static string FormatElapsedTime(float elapsedSeconds)
    {
        int totalSeconds = Mathf.FloorToInt(Mathf.Max(0f, elapsedSeconds));
        int minutes = totalSeconds / 60;
        int seconds = totalSeconds % 60;
        return string.Format("{0:00}:{1:00}", minutes, seconds);
    }

    private void UpdateNameReadinessLabel()
    {
        UpdateStatusLabel(hasName ? "Ready" : "Waiting for player name...");
    }
}