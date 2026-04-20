using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using System.Text;

public class MQTTController_Socket2 : MonoBehaviour
{
    MqttClient client;

    public string brokerIP = "192.168.0.190";
    public string topic = "lab/room1/socket2/cmd";   // 👈 SOCKET 2 TOPIC

    void Start()
    {
        client = new MqttClient(brokerIP);
        client.Connect("UnityMRTKClient_Socket2");    // 👈 unique client ID
        Debug.Log("MQTT SOCKET 2 CONNECTED");
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

        Debug.Log("MQTT SOCKET 2 SENT: " + payload);
    }
}
