using UnityEngine;
using System;
using System.Net.Sockets;
using System.Threading;
using System.IO;

public class TogglableReceiver : MonoBehaviour
{
    [Header("Network Settings")]
    public string piIp = "169.254.1.10";
    public int currentPort = 8085; // Default to Inspector

    private Renderer meshRenderer;
    private Texture2D texLeft, texRight;
    private byte[] dataLeft, dataRight;
    private bool readyLeft, readyRight;
    
    private Thread networkThread;
    private bool isRunning = true;

    void Start()
    {
        meshRenderer = GetComponent<Renderer>();
        texLeft = new Texture2D(1280, 720, TextureFormat.RGB24, false);
        texRight = new Texture2D(1280, 720, TextureFormat.RGB24, false);
        
        meshRenderer.material.SetTexture("_LeftTex", texLeft);
        meshRenderer.material.SetTexture("_RightTex", texRight);

        StartNetworkThread();
    }

    // This is the function your UI buttons will call
    public void SwitchCameraSource(int newPort)
    {
        if (currentPort == newPort) return;
        
        Debug.Log($"V3: Switching from {currentPort} to {newPort}...");
        currentPort = newPort;
        
        // Restart the connection
        StopNetworkThread();
        StartNetworkThread();
    }

    private void StartNetworkThread()
    {
        isRunning = true;
        networkThread = new Thread(ReceiveLoop);
        networkThread.IsBackground = true;
        networkThread.Start();
    }

    private void StopNetworkThread()
    {
        isRunning = false;
        if (networkThread != null && networkThread.IsAlive)
        {
            networkThread.Abort();
        }
    }

    void Update()
    {
        lock(this) {
            if (readyLeft && readyRight) {
                texLeft.LoadImage(dataLeft);
                texRight.LoadImage(dataRight);
                readyLeft = readyRight = false;
            }
        }
    }

    void ReceiveLoop()
    {
        TcpClient client = null;
        try {
            client = new TcpClient(piIp, currentPort);
            NetworkStream stream = client.GetStream();
            BinaryReader reader = new BinaryReader(stream);

            while (isRunning) {
    // 1. Read the type tag
    byte[] tag = reader.ReadBytes(1);
    if (tag.Length == 0) break;
    byte type = tag[0];

    // 2. Read the size
    byte[] sizeBytes = reader.ReadBytes(4);
    if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);
    int size = BitConverter.ToInt32(sizeBytes, 0);

    // 3. READ THE FULL IMAGE DATA (The Flicker Fix)
    // Using reader.ReadBytes(size) is safer than stream.Read() 
    // because it waits until the whole image arrives.
    byte[] frameData = reader.ReadBytes(size);

    lock(this) {
        if (type == 76) { dataLeft = frameData; readyLeft = true; }
        else if (type == 82) { dataRight = frameData; readyRight = true; }
    }
}
        } catch (Exception e) {
            if (isRunning) Debug.LogWarning($"V3 Switcher Error: {e.Message}");
        } finally {
            client?.Close();
        }
    }

    void OnDestroy() => StopNetworkThread();
}