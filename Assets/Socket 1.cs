using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using System.Text;

public class MQTTController : MonoBehaviour
{
    MqttClient client;

    public string brokerIP = "192.168.0.190";
    public string topic = "lab/room1/socket1/cmd";

    void Start()
    {
        client = new MqttClient(brokerIP);
        client.Connect("UnityMRTKClient");
        Debug.Log("MQTT CONNECTED");
    }

    public void SocketOn()
    {
        Publish("on");
    }

    public void SocketOff()
    {
        Publish("off");
    }

    private void Publish(string payload)
    {
        if (!client.IsConnected) return;

        client.Publish(
            topic,
            Encoding.UTF8.GetBytes(payload)
        );

        Debug.Log("MQTT SENT: " + payload);
    }
}
