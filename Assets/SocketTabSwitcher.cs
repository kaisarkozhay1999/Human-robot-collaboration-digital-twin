using UnityEngine;

public class SocketTabSwitcher : MonoBehaviour
{
    public GameObject buttonsTab;
    public GameObject detailsTab;

    public void ShowDetails()
    {
        buttonsTab.SetActive(false);
        detailsTab.SetActive(true);
    }

    public void ShowButtons()
    {
        detailsTab.SetActive(false);
        buttonsTab.SetActive(true);
    }
}
