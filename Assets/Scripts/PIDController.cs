using UnityEngine;
using System.Collections;

public class PIDController : MonoBehaviour
{
    float CTE_old = 0f;
    float CTE_sum = 0f;

    public float k = 0f;
    public float t = 0f;

    

    public float GetSteerFactorFromPIDController(float CTE)
    {
        //The steering factor
        float alpha = 0f;


        //P
        alpha = (float)(0.6 * k * CTE);


        //I
        CTE_sum += Time.fixedDeltaTime * CTE;

        //Sometimes better to just sum the last errors
        float averageAmount = 20f;

        CTE_sum = CTE_sum + ((CTE - CTE_sum) / averageAmount);

        alpha += (float)((1.2*k/t) * CTE_sum);


        //D
        float d_dt_CTE = (CTE - CTE_old) / Time.fixedDeltaTime;

        alpha += (float)(0.075*k*t * d_dt_CTE);

        CTE_old = CTE;


        return alpha;
    }
}