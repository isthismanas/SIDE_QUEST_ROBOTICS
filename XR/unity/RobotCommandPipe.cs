using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System;
using System.Collections.Generic;
using UnityEngine.Events;
using System.Collections;

public class RobotCommandPipe : MonoBehaviour
{
    [Header("Network Settings")]
    public string piIp = "169.254.1.10";
    public int commandPort = 8088;

    [Header("Nudge Settings (mm per nudge)")]
    public float nudgeStep = 3f;

    [Header("Nudge Repeat (hold-to-nudge)")]
    [Tooltip("While holding, send a nudge every X milliseconds.")]
    public float nudgeRepeatIntervalMs = 200f;

    [Tooltip("Minimum time allowed between ANY two nudges (tap or hold). Prevents spam queueing.")]
    public float nudgeCooldownMs = 150f;

    [Header("Debug")]
    public bool logCommands = false;

    [Header("Zone Status (from Pi)")]
    [Tooltip("Latest zone received from Pi (ZONE GREEN/YELLOW/RED).")]
    public string currentZone = "GREEN";

    [Tooltip("Invoked when zone changes (GREEN/YELLOW/RED).")]
    public UnityEvent<string> OnZoneChanged;

    [Header("Reconnect")]
    public float reconnectIntervalSeconds = 1.0f;

    private TcpClient client;
    private NetworkStream stream;

    private readonly byte[] _recvBuf = new byte[1024];
    private readonly StringBuilder _recvAccum = new StringBuilder(2048);

    private readonly Queue<string> _mainThreadMessages = new Queue<string>();
    private readonly object _queueLock = new object();

    private float _nextReconnectTime = 0f;
    private bool _isConnectingOrConnected = false;

    // --- Nudge anti-spam / hold-to-repeat ---
    private Coroutine _nudgeRoutine = null;
    private float _lastNudgeTime = -999f;

    void Start()
    {
        Connect();
    }

    void Update()
    {
        // Handle queued incoming messages on the main thread
        while (true)
        {
            string msg = null;
            lock (_queueLock)
            {
                if (_mainThreadMessages.Count > 0)
                    msg = _mainThreadMessages.Dequeue();
            }
            if (msg == null) break;

            HandleIncomingLine(msg);
        }

        // If disconnected, attempt periodic reconnect
        if (!_isConnectingOrConnected && Time.unscaledTime >= _nextReconnectTime)
        {
            _nextReconnectTime = Time.unscaledTime + reconnectIntervalSeconds;
            Connect();
        }
    }

    void Connect()
    {
        if (_isConnectingOrConnected) return;

        _isConnectingOrConnected = true;

        try
        {
            client?.Close();
            client = new TcpClient(piIp, commandPort);
            stream = client.GetStream();

            if (logCommands)
                Debug.Log("[RobotCommandPipe] Command Highway Linked!");

            // Start async receive
            BeginRead();
        }
        catch (Exception e)
        {
            if (logCommands)
                Debug.LogWarning("[RobotCommandPipe] Command Hub Offline: " + e.Message);

            stream = null;
            client = null;
            _isConnectingOrConnected = false;
        }
    }

    void BeginRead()
    {
        try
        {
            if (stream == null || !stream.CanRead) return;
            stream.BeginRead(_recvBuf, 0, _recvBuf.Length, OnRead, null);
        }
        catch (Exception e)
        {
            if (logCommands)
                Debug.LogWarning("[RobotCommandPipe] BeginRead failed: " + e.Message);
            MarkDisconnected();
        }
    }

    void OnRead(IAsyncResult ar)
    {
        try
        {
            if (stream == null)
            {
                MarkDisconnected();
                return;
            }

            int n = stream.EndRead(ar);
            if (n <= 0)
            {
                // Remote closed
                MarkDisconnected();
                return;
            }

            string chunk = Encoding.UTF8.GetString(_recvBuf, 0, n);
            _recvAccum.Append(chunk);

            // Parse newline-delimited messages
            while (true)
            {
                int idx = _recvAccum.ToString().IndexOf('\n');
                if (idx < 0) break;

                string line = _recvAccum.ToString(0, idx).Trim();
                _recvAccum.Remove(0, idx + 1);

                if (!string.IsNullOrEmpty(line))
                {
                    lock (_queueLock)
                    {
                        _mainThreadMessages.Enqueue(line);
                    }
                }
            }

            // Continue reading
            BeginRead();
        }
        catch (Exception e)
        {
            if (logCommands)
                Debug.LogWarning("[RobotCommandPipe] Read failed: " + e.Message);
            MarkDisconnected();
        }
    }

    void HandleIncomingLine(string line)
    {
        // Minimal parser: ZONE <COLOR>
        if (line.StartsWith("ZONE ", StringComparison.OrdinalIgnoreCase))
        {
            string zone = line.Substring(5).Trim().ToUpperInvariant();
            if (zone != "GREEN" && zone != "YELLOW" && zone != "RED")
                return;

            if (zone != currentZone)
            {
                currentZone = zone;
                OnZoneChanged?.Invoke(currentZone);

                if (logCommands)
                    Debug.Log("[RobotCommandPipe] Zone updated: " + currentZone);
            }

            return;
        }


        // Optional: log other inbound messages only if debug enabled
        if (logCommands)
            Debug.Log("[RobotCommandPipe] RX: " + line);
    }


    // =========================
    // High-Level Commands
    // =========================
    public void SendStart() => SendAction("START");
    public void SendCommit() => SendAction("COMMIT");  // server maps COMMIT->DROP
    public void SendDrop() => SendAction("DROP");
    public void SendFix() => SendAction("FIX");
    public void SendCancel() => SendAction("CANCEL");
    public void SendHome() => SendAction("HOME");

    // =========================
    // Gripper Commands
    // =========================
    public void SendGripToggle() => SendAction("GRIP_TOGGLE");
    public void SendGripOpen() => SendAction("GRIP_OPEN");
    public void SendGripClose() => SendAction("GRIP_CLOSE");

    // =========================
    // Nudge Commands (single-tap)
    // =========================
    public void NudgeRight() => SendNudge(nudgeStep, 0f);
    public void NudgeLeft() => SendNudge(-nudgeStep, 0f);
    public void NudgeUp() => SendNudge(0f, nudgeStep);
    public void NudgeDown() => SendNudge(0f, -nudgeStep);


    // =========================
    // Hold-to-nudge (UI PointerDown/Up hooks)
    // =========================
    public void BeginNudgeRight() => StartHoldNudge(nudgeStep, 0f);
    public void EndNudgeRight() => StopHoldNudge();

    public void BeginNudgeLeft() => StartHoldNudge(-nudgeStep, 0f);
    public void EndNudgeLeft() => StopHoldNudge();

    public void BeginNudgeUp() => StartHoldNudge(0f, nudgeStep);
    public void EndNudgeUp() => StopHoldNudge();

    public void BeginNudgeDown() => StartHoldNudge(0f, -nudgeStep);
    public void EndNudgeDown() => StopHoldNudge();

    void SendNudge(float dx, float dy)
    {
        // Cooldown guard (prevents spam queueing)
        float cooldownS = Mathf.Max(0f, nudgeCooldownMs / 1000f);
        if (Time.unscaledTime - _lastNudgeTime < cooldownS)
            return;

        _lastNudgeTime = Time.unscaledTime;

        SendAction($"NUDGE {dx} {dy}");
    }

    void StartHoldNudge(float dx, float dy)
    {
        StopHoldNudge();
        _nudgeRoutine = StartCoroutine(HoldNudgeLoop(dx, dy));
    }

    void StopHoldNudge()
    {
        if (_nudgeRoutine != null)
        {
            StopCoroutine(_nudgeRoutine);
            _nudgeRoutine = null;
        }
    }

    IEnumerator HoldNudgeLoop(float dx, float dy)
    {
        float intervalS = Mathf.Max(0.01f, nudgeRepeatIntervalMs / 1000f);
        while (true)
        {
            SendNudge(dx, dy);
            yield return new WaitForSecondsRealtime(intervalS);
        }
    }

    void MarkDisconnected()
    {
        try { stream?.Close(); } catch { }
        try { client?.Close(); } catch { }
        stream = null;
        client = null;
        _isConnectingOrConnected = false;
    }

    // =========================
    // Core Sender
    // =========================
    public void SendAction(string cmd)
    {
        cmd = cmd.Trim();

        try
        {
            // If stream is missing or dead, try reconnect once
            if (stream == null || !stream.CanWrite)
            {
                if (logCommands)
                    Debug.LogWarning($"[RobotCommandPipe] Stream not writable. Reconnecting before sending: {cmd}");
                MarkDisconnected();
                Connect();
            }

            // If still no stream, bail with a clear warning
            if (stream == null)
            {
                if (logCommands)
                    Debug.LogWarning($"[RobotCommandPipe] No stream. Command NOT sent: {cmd}");
                return;
            }

            byte[] data = Encoding.UTF8.GetBytes(cmd + "\n");
            stream.Write(data, 0, data.Length);
            stream.Flush();

            if (logCommands)
                Debug.Log($"[RobotCommandPipe] Command Sent: {cmd}");
        }
        catch (Exception e)
        {
            if (logCommands)
                Debug.LogWarning("[RobotCommandPipe] Send failed, reconnecting: " + e.Message);

            MarkDisconnected();
            // Next Update() will reconnect
        }
    }

    void OnDisable()
    {
        // Safety: stop hold nudging if object disables mid-hold
        StopHoldNudge();
    }

    void OnDestroy()
    {
        if (logCommands)
            Debug.Log("[RobotCommandPipe] Shutting down TCP client.");

        StopHoldNudge();
        MarkDisconnected();
    }
}