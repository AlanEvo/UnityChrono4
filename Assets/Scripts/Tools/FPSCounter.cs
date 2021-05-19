using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class FPSCounter : MonoBehaviour
{

    private string _fpsText;
    [SerializeField] private float _hudRefreshRate = 1f;

    private float _timer;

    void Start()
    {
       // _fpsText = GetComponent<Text>();
    }

    private void Update()
    {
        if (Time.unscaledTime > _timer)
        {
            int fps = (int)(1f / Time.unscaledDeltaTime);
            _fpsText = fps.ToString();
            _timer = Time.unscaledTime + _hudRefreshRate;
        }
    }

    private void OnGUI()
    {
        float guiWidth = Screen.width / 2f;


        GUI.Label(new Rect(Screen.width - 200 - guiWidth, 35, 40, 150), "Fps : " + _fpsText);


    }
}