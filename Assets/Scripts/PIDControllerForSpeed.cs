using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PIDControllerForSpeed : MonoBehaviour
{
    float error_old = 0f;
    float error_sum = 0f;

    //PID parameters
    public float P = 0f;
    public float I = 0f;
    public float D = 0f;

    public float GetThrottleFactorFromPIDController(float error)
    {
        float result = 0f;

        //P
        result = P * error;

        //I
        error_sum += Time.fixedDeltaTime * error;
        result += I * error_sum;

        //D
        float d_error_dt = (error - error_old) / Time.fixedDeltaTime;
        result += D * d_error_dt;

        error_old = error;

        return result;
    }
}