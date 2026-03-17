using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using System.Text;

public class MQTTController_Socket3 : MonoBehaviour
{
    MqttClient client;

    public string brokerIP = "192.168.0.190";
    public string topic = "lab/room1/socket3/cmd";   // 👈 SOCKET 3 TOPIC

    void Start()
    {
        client = new MqttClient(brokerIP);
        client.Connect("UnityMRTKClient_Socket3");    // 👈 unique client ID
        Debug.Log("MQTT SOCKET 3 CONNECTED");
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

        Debug.Log("MQTT SOCKET 3 SENT: " + payload);
    }
}
