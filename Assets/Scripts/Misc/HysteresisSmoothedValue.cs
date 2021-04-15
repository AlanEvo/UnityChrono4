using UnityEngine;

public class HysteresisSmoothedValue
{
    public float Value
    {
        get
        {
            return _value;
        }
    }

    public HysteresisSmoothedValue(float initial, float riseTime, float fallTime)
    {
        _value = initial;
        _riseTime = riseTime;
        _fallTime = fallTime;
    }

    public void Tick(float target)
    {
        float speed = target < _value ? _fallTime : _riseTime;
        _value = Mathf.SmoothDamp(_value, target, ref _velocity, speed);
    }

    private float _value;
    private float _riseTime = 1;
    private float _fallTime = 1;
    private float _velocity = 0;
}
