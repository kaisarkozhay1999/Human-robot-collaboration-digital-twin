using System;
using System.IO;
using System.Net;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

public class MJPEGViewer : MonoBehaviour
{
    [Header("Stream Settings")]
    public string streamURL = "http://192.168.0.190:8080/?action=stream";
    
    [Header("UI Reference")]
    public RawImage targetDisplay;

    private Texture2D streamTexture;
    private Thread streamThread;
    private byte[] currentFrameBytes;
    private readonly object frameLock = new object();
    private bool isRunning = false;

    // --- Optimization Buffers ---
    // 1MB buffer should be more than enough for a single JPEG frame at 1080p
    private const int MaxFrameSize = 1024 * 1024; 
    private byte[] frameAssemblyBuffer; 
    private byte[] networkReadBuffer;

    void Start()
    {
        streamTexture = new Texture2D(2, 2, TextureFormat.RGB24, false);
        if (targetDisplay != null) targetDisplay.texture = streamTexture;

        // Pre-allocate our buffers once when the script starts
        frameAssemblyBuffer = new byte[MaxFrameSize];
        networkReadBuffer = new byte[65536]; // 64KB chunks for network reading

        isRunning = true;
        streamThread = new Thread(ReadStream) { IsBackground = true };
        streamThread.Start();
    }

    private void ReadStream()
    {
        while (isRunning)
        {
            try
            {
                HttpWebRequest request = (HttpWebRequest)WebRequest.Create(streamURL);
                request.Timeout = 5000;

                using (WebResponse response = request.GetResponse())
                using (Stream stream = response.GetResponseStream())
                {
                    bool isRecording = false;
                    int frameSize = 0;

                    while (isRunning)
                    {
                        // Read into our pre-allocated network buffer
                        int bytesRead = stream.Read(networkReadBuffer, 0, networkReadBuffer.Length);
                        if (bytesRead == 0) break; // Stream stopped

                        for (int i = 0; i < bytesRead; i++)
                        {
                            byte b = networkReadBuffer[i];

                            if (!isRecording)
                            {
                                // Look for Start of Image (SOI) marker: FF D8
                                if (frameSize == 0 && b == 0xFF)
                                {
                                    frameAssemblyBuffer[frameSize++] = b;
                                }
                                else if (frameSize == 1)
                                {
                                    if (b == 0xD8)
                                    {
                                        isRecording = true;
                                        frameAssemblyBuffer[frameSize++] = b;
                                    }
                                    else
                                    {
                                        frameSize = 0;
                                        if (b == 0xFF) frameAssemblyBuffer[frameSize++] = b;
                                    }
                                }
                            }
                            else
                            {
                                // Prevent buffer overflow if the frame is unexpectedly huge
                                if (frameSize >= MaxFrameSize)
                                {
                                    Debug.LogWarning("Frame exceeded maximum buffer size. Dropping frame.");
                                    isRecording = false;
                                    frameSize = 0;
                                    continue;
                                }

                                frameAssemblyBuffer[frameSize++] = b;
                                
                                // Look for End of Image (EOI) marker: FF D9
                                if (frameSize >= 2 && 
                                    frameAssemblyBuffer[frameSize - 2] == 0xFF && 
                                    frameAssemblyBuffer[frameSize - 1] == 0xD9)
                                {
                                    // Allocate only the exact size needed for the final frame
                                    byte[] finalFrame = new byte[frameSize];
                                    
                                    // Extremely fast memory copy
                                    Buffer.BlockCopy(frameAssemblyBuffer, 0, finalFrame, 0, frameSize);
                                    
                                    lock (frameLock)
                                    {
                                        currentFrameBytes = finalFrame;
                                    }
                                    
                                    // Reset for the next frame
                                    isRecording = false;
                                    frameSize = 0; 
                                }
                            }
                        }
                    }
                }
            }
            catch (Exception e)
            {
                if (isRunning)
                {
                    Debug.LogWarning($"Stream error: {e.Message}. Retrying...");
                    Thread.Sleep(2000);
                }
            }
        }
    }

    void Update()
    {
        byte[] frameToLoad = null;

        lock (frameLock)
        {
            if (currentFrameBytes != null)
            {
                frameToLoad = currentFrameBytes;
                currentFrameBytes = null; 
            }
        }

        if (frameToLoad != null)
        {
            // Unity handles the decoding of the JPEG byte array here
            streamTexture.LoadImage(frameToLoad);
        }
    }

    void OnDestroy()
    {
        isRunning = false;
        if (streamThread != null && streamThread.IsAlive)
        {
            streamThread.Join(500);
        }
    }
}