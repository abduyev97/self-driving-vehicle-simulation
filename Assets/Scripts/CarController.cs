using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using HealthbarGames;


[System.Serializable]
public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}


public class CarController : MonoBehaviour
{
    public List<AxleInfo> axleInfos;

    //Car data
    float maxMotorTorque = 200f;
    float maxSteeringAngle = 40f;
    public float motorTorque = 150f;
    public float brakeTorque = 1200f;
    public Rigidbody car;
    private NodeSpeed[] m_velocityList;
    //The difference between the center of the car and the position where we steer
    public float centerSteerDifference;
    //The position where the car is steering
    private Vector3 steerPosition;
    private float m_throttle = 0f;
    // true, if the vehicle is increasing the speed.
    private bool m_isAccelerating;
    private TLWarner m_currentTLwarner;
    private float m_currentTargetVelocity;
    //All waypoints
    private List<Transform> allWaypoints;
    //The current index of the list with all waypoints
    private int currentWaypointIndex = 0;
    //The waypoint we are going towards and the waypoint we are going from
    private Vector3 currentWaypoint;
    private Vector3 previousWaypoint;
    private bool avoiding = false;

    public Transform path;

    //Average the steering angles to simulate the time it takes to turn the wheel
    float averageSteeringAngle = 0f;

    [Header("Sensors")]
    public float sensorLength = 10f;
    public Vector3 FrontSensorPosition = new Vector3(1.8f, 0.5f, 2);
    public float FrontSideSensorsOffset = 0.47f;
    public float FrontSideSensorAngle = 30f;

    PIDController PIDControllerScript;
    private PIDControllerForSpeed m_PIDControllerForSpeed;


    void Start()
    {
        Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();
        allWaypoints = new List<Transform>();

        for (int i = 0; i < pathTransforms.Length; i++)
        {
            if (pathTransforms[i] != path.transform)
            {
                allWaypoints.Add(pathTransforms[i]);
            }
        }
        

        //Init the waypoints
        currentWaypoint = allWaypoints[currentWaypointIndex].position;

        m_velocityList = path.GetComponentsInChildren<NodeSpeed>();

        previousWaypoint = GetPreviousWaypoint();

        PIDControllerScript = GetComponent<PIDController>();
        m_PIDControllerForSpeed = GetComponent<PIDControllerForSpeed>();
    }

    //Finds the corresponding visual wheel, correctly applies the transform
    void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0)
        {
            return;
        }

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }

    void Update()
    {
        //So we can experiment with the position where the car is checking if it should steer left/right
        //doesn't have to be where the wheels are - especially if we are reversing
        steerPosition = transform.position + transform.forward * centerSteerDifference;

        //Check if we should change waypoint
        if (Math.HasPassedWaypoint(steerPosition, previousWaypoint, currentWaypoint))
        {
            currentWaypointIndex += 1;

            if (currentWaypointIndex == allWaypoints.Count)
            {
                currentWaypointIndex = 0;
            }

            currentWaypoint = allWaypoints[currentWaypointIndex].position;

            previousWaypoint = GetPreviousWaypoint();
        }
    }

    private void Accelerate()
    {
        
        if (m_isAccelerating)
        {

            foreach (AxleInfo axleInfo in axleInfos)
            {
                //Debug.Log("accelerate");
                axleInfo.leftWheel.brakeTorque = 0;
                axleInfo.rightWheel.brakeTorque = 0;
                axleInfo.leftWheel.motorTorque = motorTorque * m_throttle;
                axleInfo.rightWheel.motorTorque = motorTorque * m_throttle;

                ApplyLocalPositionToVisuals(axleInfo.leftWheel);
                ApplyLocalPositionToVisuals(axleInfo.rightWheel);
            }
        }

    }

    private void Brake()
    {
        if (!m_isAccelerating)
        {

            foreach (AxleInfo axleInfo in axleInfos)
            {
                //Debug.Log("braking");
                axleInfo.leftWheel.brakeTorque = brakeTorque * m_throttle;
                axleInfo.rightWheel.brakeTorque = brakeTorque * m_throttle;
                axleInfo.leftWheel.motorTorque = 0;
                axleInfo.rightWheel.motorTorque = 0;

                ApplyLocalPositionToVisuals(axleInfo.leftWheel);
                ApplyLocalPositionToVisuals(axleInfo.rightWheel);
            }
        }
    }

    private void CalculateThrottleValue()
    {
        float currentVelocity = car.velocity.magnitude * 3.6f;
        float speedError = m_currentTargetVelocity - currentVelocity;

        //Debug.Log(speedError);

        m_throttle = m_PIDControllerForSpeed.GetThrottleFactorFromPIDController(speedError);
        m_throttle = Mathf.Clamp(m_throttle, -1, 1);
        if (m_throttle > 0)
        {
            m_isAccelerating = true;
        }
        else
        {
            m_isAccelerating = false; 
            m_throttle *= -1;
        }
    }

    private void CheckTrafficLight()
    {
        
        if (m_currentTLwarner != null)
        {
            if ((m_currentTLwarner.Left_TrafficLight.RedHalo.activeSelf &&
                !m_currentTLwarner.Left_TrafficLight.YellowHalo.activeSelf) ||
                (m_currentTLwarner.Left_TrafficLight.YellowHalo.activeSelf))
            {
                Debug.Log("Checked lights");
                m_currentTargetVelocity = 0;

            }
        }
    }

    //Get the waypoint before the current waypoint we are driving towards
    Vector3 GetPreviousWaypoint()
    {
        previousWaypoint = Vector3.zero;

        if (currentWaypointIndex - 1 < 0)
        {
            previousWaypoint = allWaypoints[allWaypoints.Count - 1].position;
        }
        else
        {
            previousWaypoint = allWaypoints[currentWaypointIndex - 1].position;
        }

        return previousWaypoint;
    }

    private void Steer()
    {
        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
            {
                if (!avoiding)
                {

                    axleInfo.leftWheel.steerAngle = averageSteeringAngle;
                    axleInfo.rightWheel.steerAngle = averageSteeringAngle;
                }
            }
        }
    }
    void FixedUpdate()
    {
        m_currentTargetVelocity = m_velocityList[currentWaypointIndex].targetSpeed;
        CreateSensors();
        CheckTrafficLight();
        CalculateSteering();
        CalculateThrottleValue();

        //
        //Apply everything to the car 
        //
        //foreach (AxleInfo axleInfo in axleInfos)
        //{
        //    if (axleInfo.steering)
        //    {
        //        if (!avoiding)
        //        {

        //            axleInfo.leftWheel.steerAngle = averageSteeringAngle;
        //            axleInfo.rightWheel.steerAngle = averageSteeringAngle;
        //        }
        //    }
        //    if (axleInfo.motor)
        //    {
        //        if ((int)(allWaypoints[currentWaypointIndex].GetComponent<NodeSpeed>().targetSpeed) < (car.velocity.magnitude * 3.6)){
        //            axleInfo.leftWheel.brakeTorque = 0;
        //            axleInfo.rightWheel.brakeTorque = 0;
        //        }
        //        else {
        //            axleInfo.leftWheel.motorTorque = motorTorque;
        //            axleInfo.rightWheel.motorTorque = motorTorque;
        //        }

        //    }

        //    ApplyLocalPositionToVisuals(axleInfo.leftWheel);
        //    ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        //}
        Steer();
        Accelerate();
        Brake();
        

    }

    private void CreateSensors()
    {
        RaycastHit hit;
        Vector3 sensorStartPos = transform.position + transform.forward * FrontSensorPosition.x + transform.up * FrontSensorPosition.y;
        float avoidMultiplier = 0;
        avoiding = false;
        

        //Front Right Side Sensor
        sensorStartPos += transform.right * FrontSideSensorsOffset;
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (!hit.collider.CompareTag("Ground") && !hit.collider.CompareTag("Warner") && !hit.collider.CompareTag("TLPosition"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
                avoidMultiplier -= 1f;
            }
        }

        //Front Right Side Angle Sensor
        else if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(FrontSideSensorAngle, transform.up) * transform.forward, out hit, sensorLength))
        {
            if (!hit.collider.CompareTag("Ground") && !hit.collider.CompareTag("Warner") && !hit.collider.CompareTag("TLPosition"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
                avoidMultiplier -= 0.5f;
            }
        }

        //Front Left Side Sensor
        sensorStartPos -= 2 * transform.right * FrontSideSensorsOffset;
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (!hit.collider.CompareTag("Ground") && !hit.collider.CompareTag("Warner") && !hit.collider.CompareTag("TLPosition"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
                avoidMultiplier += 1f;
            }
        }

        //Front Left Side Angle Sensor
        else if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(-FrontSideSensorAngle, transform.up) * transform.forward, out hit, sensorLength))
        {
            if (!hit.collider.CompareTag("Ground") && !hit.collider.CompareTag("Warner") && !hit.collider.CompareTag("TLPosition"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
                avoidMultiplier += 0.5f;
            }
        }
        Debug.Log(avoidMultiplier);
        // Front Center Sensor
        if(avoidMultiplier == 0)
        {
            if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
            {
                if (!hit.collider.CompareTag("Ground") && !hit.collider.CompareTag("Warner") && !hit.collider.CompareTag("TLPosition"))
                {
                    Debug.DrawLine(sensorStartPos, hit.point);
                    avoiding = true;
                    if(hit.normal.x < 0)
                    {
                        avoidMultiplier = -1;
                    }
                    else
                    {
                        avoidMultiplier = 1;
                    }
                }
            }
        }
        if (avoiding)
        {
            foreach(AxleInfo axleInfo in axleInfos)
            {
                if (axleInfo.steering)
                {
                    axleInfo.leftWheel.steerAngle = maxSteeringAngle * avoidMultiplier;
                    axleInfo.rightWheel.steerAngle = maxSteeringAngle * avoidMultiplier;
                }
                ApplyLocalPositionToVisuals(axleInfo.leftWheel);
                ApplyLocalPositionToVisuals(axleInfo.rightWheel);
            }

        }

    }

    private void CalculateSteering()
    {
        //Get the cross track error, which is what we want to minimize with the pid controller
        float CTE = Math.GetCrossTrackError(steerPosition, previousWaypoint, currentWaypoint);

        //But we still need a direction to steer
        CTE *= Math.SteerDirection(transform, steerPosition, currentWaypoint);

        float steeringAngle = PIDControllerScript.GetSteerFactorFromPIDController(CTE);

        //Limit the steering angle
        steeringAngle = Mathf.Clamp(steeringAngle, -maxSteeringAngle, maxSteeringAngle);

        //Average the steering angles to simulate the time it takes to turn the steering wheel
        float averageAmount = 30f;

        averageSteeringAngle = averageSteeringAngle + ((steeringAngle - averageSteeringAngle) / averageAmount);
    }

    void OnTriggerEnter(Collider collider)
    {
        if (collider.tag == "Warner")
        {
            m_currentTLwarner = collider.GetComponent<TLWarner>();
            
        }
        else if (collider.tag == "TLPosition")
        {
            m_currentTLwarner = null;
        }
    }
    //void OnTriggerExit(Collider collider)
    //{
    //    if(collider.tag == "Warner")
    //    {
    //        m_currentTLwarner = null;
    //    }
    //}

}