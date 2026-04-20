using UnityEngine;

public class DeviceUIController : MonoBehaviour
{
    public GameObject smallUI;
    public GameObject bigUI;

    private bool isBigOpen = false;

    private void Log(string message)
    {
        Debug.Log($"[DeviceUI:{gameObject.name}] {message}");
    }

    public void ToggleBigUI()
    {
        Log("ToggleBigUI() called");

        isBigOpen = !isBigOpen;
        Log("isBigOpen = " + isBigOpen);

        Invoke(nameof(ApplyToggle), 0.05f);
    }

    private void ApplyToggle()
    {
        Log("ApplyToggle()");

        bigUI.SetActive(isBigOpen);
        smallUI.SetActive(!isBigOpen);

        if (isBigOpen)
        {
            Log("Opening Big UI");

            bigUI.transform.position =
                transform.position
                + transform.forward * 0.3f
                + Vector3.up * 0.1f;

            bigUI.transform.LookAt(Camera.main.transform);
            bigUI.transform.Rotate(0, 180, 0);
        }
        else
        {
            Log("Closing Big UI");
        }
    }

    public void CloseBigUI()
    {
        Log("CloseBigUI() called");

        isBigOpen = false;
        Invoke(nameof(ApplyClose), 0.05f);
    }

    private void ApplyClose()
    {
        Log("ApplyClose()");
        bigUI.SetActive(false);
        smallUI.SetActive(true);
    }
}
