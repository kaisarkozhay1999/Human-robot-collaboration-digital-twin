using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using System.Text;
using MixedReality.Toolkit.UX;

public class MQTTBulbController : MonoBehaviour
{
    MqttClient client;

    public string brokerIP = "192.168.0.190";
    public string topic = "lab/room1/bulb1/cmd";

    // HA expects 0–255
    int brightness = 204; // default ~80%

    int r = 255, g = 255, b = 255;

    void Start()
    {
        client = new MqttClient(brokerIP);
        client.Connect("UnityBulbClient_" + topic);
        Debug.Log("BULB MQTT CONNECTED");
    }

    // ---------- ON / OFF ----------

    public void LightOn()
    {
        Publish(BuildJson("ON"));
    }

    public void LightOff()
    {
        Publish(BuildJson("OFF"));
    }

    // ---------- BRIGHTNESS (MRTK SLIDER) ----------

    public void UpdateBrightness(SliderEventData data)
    {
        // Slider 0.0–1.0 → HA 0–255
        brightness = Mathf.RoundToInt(data.NewValue * 100f);

        Publish(BuildJson("ON"));

        Debug.Log("Brightness (slider): " + brightness);
    }

    // ---------- COLOR BUTTONS (THIS WAS MISSING) ----------

    public void SetRed()     { SetColor(255, 0, 0); }
    public void SetGreen()   { SetColor(0, 255, 0); }
    public void SetBlue()    { SetColor(0, 0, 255); }
    public void SetOrange()  { SetColor(255, 128, 0); }
    public void SetPurple()  { SetColor(180, 100, 255); }
    public void SetWhite()   { SetColor(255, 255, 255); }

    void SetColor(int red, int green, int blue)
    {
        r = red;
        g = green;
        b = blue;

        Publish(BuildJson("ON"));

        Debug.Log($"Color set: R={r} G={g} B={b}");
    }

    // ---------- MQTT ----------

    private string BuildJson(string state)
    {
        return
            "{ \"state\": \"" + state + "\", " +
            "\"brightness\": " + brightness + ", " +
            "\"color\": { \"r\": " + r + ", \"g\": " + g + ", \"b\": " + b + " } }";
    }

    private void Publish(string payload)
    {
        if (!client.IsConnected) return;

        client.Publish(topic, Encoding.UTF8.GetBytes(payload));

        Debug.Log("BULB MQTT SENT: " + payload);
    }
}
