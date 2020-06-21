using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;


public class controller : MonoBehaviour


{
    public bool autonomous_control;
    public enum FlightMode{ Vertical, Horizontal }
    public FlightMode flightMode;



    public float delta_both;
    public float delta_diff;
    public float delta_left;
    public float delta_right;

    public float RPM_inner;
    public float RPM_left;
    public float RPM_all;

    public float[] RPM = new float[4];

    public Transform pivot_aileron_left;
    public Transform pivot_aileron_right;
    // Start is called before the first frame update

    public Vector3 body_orientation;

    GameObject cg;

    GameObject vertical_stabilizer;

    GameObject airflow;


    Transform aerodynamic_frame;
    Transform inertial_frame;
    public static Transform body_frame;

    GameObject force;

     Vector3 linear_acceleration = new Vector3(0,0,0);

     Vector3 linear_velocity = new Vector3(0,0,10);

    public double[][] angular_acceleration = new double[3][] { new double[]{0}, new double[]{0}, new double[]{0}};
    public double[][] angular_velocity = new double[3][] { new double[]{0}, new double[]{0}, new double[]{0}};
    public double[][] moment = new double[3][] { new double[]{1}, new double[]{0}, new double[]{0}};

     Vector3 angular_acceleration_v3 = Vector3.zero;
    public Vector3 angular_velocity_v3 = Vector3.zero;

    float mass = .5f;

    double[][] A;

    List<Force> forces = new List<Force>();

    public static GameObject lineprefab;

    Dictionary<string, LineRenderer> force_lines = new Dictionary<string, LineRenderer>();

    // List<LineRenderer> force_lines = new List<LineRenderer>();


    List<Force> Left_Wing_Lift = new List<Force>();
    List<Force> Right_Wing_Lift = new List<Force>();
    List<Force> Left_Wing_Drag = new List<Force>();
    List<Force> Right_Wing_Drag = new List<Force>();

    List<Force> Up_Stabilizer_Lift = new List<Force>();
    List<Force> Up_Stabilizer_Drag = new List<Force>();
    List<Force> Down_Stabilizer_Lift = new List<Force>();
    List<Force> Down_Stabilizer_Drag = new List<Force>();

    Force F_AL_Lift;
    Force F_AR_Lift;
    Force F_T1;
    Force F_T2;
    Force F_T3;
    Force F_T4;
    Force FV;
    Force W;
    public float time = 0f;
    GameObject track_camera;
    public GameObject[] propellers = new GameObject[4];
    public static Transform tr;
    public Vector3 aerodynamic_angle;
    public Vector3 gains_pitch_horizontal;
    Vector3 error_pitch_horizontal;
    public Vector3 gains_altitude_horizontal;
    Vector3 error_altitude_horizontal;
    public Vector3 gains_velocity_horizontal;
    Vector3 error_velocity_horizontal;

    // PID Vertical Flight
    public Vector3 gains_altitude_vertical;
    Vector3 error_altitude_vertical;
    public Vector3 gains_xrot_vertical;
    Vector3 error_xrot_vertical;
    public Vector3 gains_yrot_vertical;
    Vector3 error_yrot_vertical;

    public float H_desired;
    public float V_desired;

     Vector3 F_total_body;
    // Constants
    const float g = 9.81f;
    // Geometrical parameters
    const float Sw = 1.4f;
    const float CL0 = 0.00690489715f;            // [-]
    const float CLalpha = 0.07587215f;          // [1/deg]
    const float CD0 = 0.0096488988f;            // [-]
    const float b = 0.0003320901f;              // [1/deg^2]   
    Transform left_tip_c4;
    Transform right_tip_c4;
    Transform root_c4;
    Transform root_c4_stabilizer;
    Transform up_tip_c4_stabilizer;
    Transform down_tip_c4_stabilizer;
    const float root_chord = 0.6608806f;
    const float tip_chord = 0.23130823f;
    const float S_stabilizer = 0.08f;
    const float Clalpha_vertical = 4.194f;

    public float pitch_desired;
    Transform p2;
    Transform p1;
    public Transform target;


    public class Force{
      
        public Vector3 position;
        public Vector3 force;
        LineRenderer lr;
        public string name;
        Transform frame;
        public float S;

        public Force(string name_, Vector3 position_, Transform frame_, Vector3 force_, float S_) 
            { 
                name = name_;
                position = position_;
                frame = frame_;
                force = force_;
                lr = Instantiate(lineprefab, tr).GetComponent<LineRenderer>();
                lr.name = name;
                S = S_;

            }
        public void update_force(Vector3 new_force){
            force =  Quaternion.Inverse(body_frame.rotation) * frame.rotation  * new_force;
            // force = Quaternion.LookRotation()
            lr.SetPosition(0, position);
            lr.SetPosition(1, position+force/200f);
        }


    }
    void Start()
    {
        if (flightMode == FlightMode.Horizontal){
            linear_velocity = Vector3.forward * V_desired;
            RPM[0] = 2250;
            RPM[1] = 2250;
            RPM[2] = 2250;
            RPM[3] = 2250;

        }
        if (flightMode == FlightMode.Vertical){
            linear_velocity = Vector3.zero;
            RPM[0] = 7535;
            RPM[1] = 7535;
            RPM[2] = 7535;
            RPM[3] = 7535;


        }
        tr = transform;
        A = MatrixInverse.MatrixInverseProgram.MatrixCreate(3,3);

        A[0][0] = 0.59006f;
        A[1][1] = 4.59936f;
        A[2][2] = 5.17967f;

        mass = 17.536f;

        vertical_stabilizer = GameObject.Find("Vertical Stabilizer Pivot");
        airflow = GameObject.Find("Airflow");
        cg = GameObject.Find("CG");
        aerodynamic_frame = GameObject.Find("Aerodynamic Frame").transform;
        inertial_frame = GameObject.Find("Inertial Frame").transform;
        track_camera = GameObject.Find("Track Drone");
        body_frame = GameObject.Find("Drone").transform;


        // topCamera = GameObject.Find("Top Camera").GetComponent<Camera>();

        lineprefab = GameObject.Find("Force");
        left_tip_c4 = GameObject.Find("Left tip (c/4)").transform;
        right_tip_c4 = GameObject.Find("Rigth tip (c/4)").transform;
        root_c4 = GameObject.Find("Root (c/4)").transform;

        up_tip_c4_stabilizer = GameObject.Find("Up tip (c/4) stabilizer").transform;
        down_tip_c4_stabilizer = GameObject.Find("Down tip (c/4) stabilizer").transform;
        root_c4_stabilizer = GameObject.Find("Root (c/4) stabilizer").transform;

        p2 = GameObject.Find("P2").transform;
        p1 = GameObject.Find("P1").transform;
        float root_chord_stabilizer = 0.08f/up_tip_c4_stabilizer.position.y;

        float totalS = 0;
        // Create Left Wing Forces
        for (var i = 0; i < 10; i++){
            float chord = root_chord + (tip_chord-root_chord)*((float)i+0.5f)/10f;
            Force lift = new Force("F_Left_Wing_Lift"+i.ToString(),  root_c4.localPosition + (left_tip_c4.localPosition-root_c4.localPosition)*((float)i+0.5f)/10f, aerodynamic_frame, Vector3.zero, chord*1.5f/10f);
            Force drag = new Force("F_Left_Wing_Drag"+i.ToString(), root_c4.localPosition + (left_tip_c4.localPosition-root_c4.localPosition)*((float)i+0.5f)/10f, aerodynamic_frame, Vector3.zero, chord*1.5f/10f);
            forces.Add(lift);
            forces.Add(drag);
            Left_Wing_Lift.Add(lift);
            Left_Wing_Drag.Add(drag);
        }
        // Create Right Wing Forces
        for (var i = 0; i < 10; i++){
            float chord = root_chord + (tip_chord-root_chord)*((float)i+0.5f)/10f;
            Force lift = new Force("F_Right_Wing_Lift"+i.ToString(), root_c4.localPosition + (right_tip_c4.localPosition-root_c4.localPosition)*((float)i+0.5f)/10f, aerodynamic_frame, Vector3.zero, chord*1.5f/10f);
            Force drag = new Force("F_Right_Wing_Drag"+i.ToString(),  root_c4.localPosition + (right_tip_c4.localPosition-root_c4.localPosition)*((float)i+0.5f)/10f, aerodynamic_frame, Vector3.zero, chord*1.5f/10f);
            forces.Add(lift);
            forces.Add(drag);
            Right_Wing_Lift.Add(lift);
            Right_Wing_Drag.Add(drag);
        }

        // Create Stabilizer Up  Forces
        for (var i = 0; i < 10; i++){
            float chord = root_chord_stabilizer + (-root_chord_stabilizer)*((float)i+0.5f)/10f;
            Force lift = new Force("F_Up_Stabilizer_Lift"+i.ToString(),  root_c4_stabilizer.localPosition + (up_tip_c4_stabilizer.localPosition-root_c4_stabilizer.localPosition)*((float)i+0.5f)/10f, aerodynamic_frame, Vector3.zero, chord*up_tip_c4_stabilizer.position.y/10f);
            Force drag = new Force("F_Up_Stabilizer_Drag"+i.ToString(), root_c4_stabilizer.localPosition + (up_tip_c4_stabilizer.localPosition-root_c4_stabilizer.localPosition)*((float)i+0.5f)/10f, aerodynamic_frame, Vector3.zero, chord*up_tip_c4_stabilizer.position.y/10f);
            // forces.Add(lift);
            // forces.Add(drag);
            // Up_Stabilizer_Lift.Add(lift);
            // Up_Stabilizer_Drag.Add(drag);
            totalS += chord*up_tip_c4_stabilizer.position.y/10f;
        }
        // Create Stabilizer Down Forces
        for (var i = 0; i < 10; i++){
            float chord = root_chord_stabilizer + (-root_chord_stabilizer)*((float)i+0.5f)/10f;
            Force lift = new Force("F_Down_Stabilizer_Lift"+i.ToString(), root_c4_stabilizer.localPosition + (down_tip_c4_stabilizer.localPosition-root_c4_stabilizer.localPosition)*((float)i+0.5f)/10f, aerodynamic_frame, Vector3.zero, chord*up_tip_c4_stabilizer.position.y/10f);
            Force drag = new Force("F_Up_Stabilizer_Drag"+i.ToString(),  root_c4_stabilizer.localPosition + (down_tip_c4_stabilizer.localPosition-root_c4_stabilizer.localPosition)*((float)i+0.5f)/10f, aerodynamic_frame, Vector3.zero, chord*up_tip_c4_stabilizer.position.y/10f);
            // forces.Add(lift);
            // forces.Add(drag);
            // Down_Stabilizer_Lift.Add(lift);
            // Down_Stabilizer_Drag.Add(drag);
            totalS += chord*up_tip_c4_stabilizer.position.y/10f;
        }

        

        F_T1 = new Force("F_T1", propellers[0].transform.localPosition, body_frame, Vector3.zero, 0f);
        F_T2 = new Force("F_T2", propellers[1].transform.localPosition, body_frame, Vector3.zero, 0f);
        F_T3 = new Force("F_T3", propellers[2].transform.localPosition, body_frame, Vector3.zero, 0f);
        F_T4 = new Force("F_T4", propellers[3].transform.localPosition, body_frame, Vector3.zero, 0f);
        F_AL_Lift = new Force("F_AL_Lift", pivot_aileron_left.localPosition + new Vector3(0,0,-0.07f), aerodynamic_frame, Vector3.zero, 0f);
        F_AR_Lift = new Force("F_AR_Lift", pivot_aileron_right.localPosition  + new Vector3(0,0,-0.07f), aerodynamic_frame, Vector3.zero, 0f);
        W = new Force("W", cg.transform.localPosition, inertial_frame, Vector3.zero, 0f);
        forces.Add(W);
        forces.Add(F_T1);
        forces.Add(F_T2);
        forces.Add(F_T3);
        forces.Add(F_T4);
        forces.Add(F_AL_Lift);
        forces.Add(F_AR_Lift);       
    }


    // Update is called once per frame
    void Update()
    {

        if (flightMode == FlightMode.Horizontal){
            aerodynamic_frame.rotation =  Quaternion.LookRotation(linear_velocity);
        }else{
            aerodynamic_frame.rotation =  Quaternion.LookRotation(transform.forward);
        }
       
        aerodynamic_angle =(Quaternion.Inverse(aerodynamic_frame.rotation) * body_frame.rotation).eulerAngles;
        Vector3 aerodynamic_vector = (Quaternion.Inverse(aerodynamic_frame.rotation) * body_frame.rotation) * Vector3.forward;
        aerodynamic_angle.x = eulerToAngle(Mathf.Atan2(aerodynamic_vector.y,aerodynamic_vector.z))*Mathf.Rad2Deg;
        // aerodynamic_angle.x = -eulerToAngle(aerodynamic_angle.x);
        aerodynamic_angle.y =  eulerToAngle(Mathf.Atan2(aerodynamic_vector.x,aerodynamic_vector.z))*Mathf.Rad2Deg;
        time += Time.deltaTime;

        if (autonomous_control){

                float offset = -0.4248f;

            if (flightMode == FlightMode.Horizontal){
                Vector3 target_vec =  Vector3.forward*40 + Vector3.up*(H_desired-transform.position.y);
                target.position = transform.position + target_vec;
                Vector3 forward_direction = aerodynamic_frame.forward;
                float v_ref =  -Mathf.Atan2(target_vec.y, target_vec.z)* Mathf.Rad2Deg;
                float v = -Mathf.Atan2(forward_direction.y,forward_direction.z) * Mathf.Rad2Deg;
                float kappa =  v_ref-v;
            if (error_pitch_horizontal.y != 0){error_pitch_horizontal.x =((kappa)-error_pitch_horizontal.y)/Time.deltaTime;}
                error_pitch_horizontal.y  =(kappa);
                error_pitch_horizontal.z += error_pitch_horizontal.y * Time.deltaTime;  
                delta_both = Mathf.Min(Mathf.Max((error_pitch_horizontal.x * gains_pitch_horizontal.x +  error_pitch_horizontal.y *gains_pitch_horizontal.y + error_pitch_horizontal.z*gains_pitch_horizontal.z)+offset, -3f+offset), 3f+offset);

                // if (error_altitude_horizontal.y != 0){error_altitude_horizontal.x =((H_desired-transform.position.y)-error_altitude_horizontal.y)/Time.deltaTime;}
                // error_altitude_horizontal.y  = (H_desired-transform.position.y);  
                // error_altitude_horizontal.z += error_altitude_horizontal.y * Time.deltaTime;  
                // delta_both = Mathf.Min( Mathf.Max(-(error_altitude_horizontal.x * gains_altitude_horizontal.x +  error_altitude_horizontal.y *gains_altitude_horizontal.y + error_altitude_horizontal.z*gains_altitude_horizontal.z)+offset, -.3f+offset), .3   f+offset);
            
                if (error_velocity_horizontal.y != 0){error_velocity_horizontal.x =((V_desired -linear_velocity.magnitude)-error_velocity_horizontal.y)/Time.deltaTime;}
                error_velocity_horizontal.y  = (V_desired-linear_velocity.magnitude);  
                error_velocity_horizontal.z += error_velocity_horizontal.y * Time.deltaTime;
                RPM_all = Mathf.Min( Mathf.Max( (error_velocity_horizontal.x * gains_velocity_horizontal.x +  error_velocity_horizontal.y *gains_velocity_horizontal.y + error_velocity_horizontal.z*gains_velocity_horizontal.z), -500f), 500f);

            }
            if (flightMode == FlightMode.Vertical){
                Vector3 up_direction = Quaternion.Inverse(transform.rotation) * inertial_frame.rotation * Vector3.up;
                float error_xrot = Mathf.Atan2(up_direction.y,up_direction.z) * Mathf.Rad2Deg;

                if (error_xrot_vertical.y != 0){error_xrot_vertical.x =((error_xrot)-error_xrot_vertical.y)/Time.deltaTime;}
                error_xrot_vertical.y  = error_xrot;  
                error_xrot_vertical.z += error_xrot_vertical.y * Time.deltaTime;  
                // delta_both = Mathf.Min( Mathf.Max( -(error_xrot_vertical.x * gains_xrot_vertical.x +  error_xrot_vertical.y *gains_xrot_vertical.y + error_xrot_vertical.z*gains_xrot_vertical.z), -5f), 5f);
            
                if (error_altitude_vertical.y != 0){error_altitude_vertical.x =((H_desired -transform.position.y)-error_altitude_vertical.y)/Time.deltaTime;}
                error_altitude_vertical.y  = (H_desired-transform.position.y);  
                error_altitude_vertical.z += error_altitude_vertical.y * Time.deltaTime;
                RPM_all = Mathf.Min( Mathf.Max( (error_altitude_vertical.x * gains_altitude_vertical.x +  error_altitude_vertical.y *gains_altitude_vertical.y + error_altitude_vertical.z*gains_altitude_vertical.z), -1000), 1000);            
                }
    
                Vector3 right_direction = Quaternion.Inverse(transform.rotation) * inertial_frame.rotation * Vector3.right;
                float error_yrot = Mathf.Atan2(right_direction.y,right_direction.x) * Mathf.Rad2Deg;
                if (error_yrot_vertical.y != 0){error_yrot_vertical.x =((error_yrot)-error_yrot_vertical.y)/Time.deltaTime;}
                error_yrot_vertical.y  = error_yrot;  
                error_yrot_vertical.z += error_yrot_vertical.y * Time.deltaTime;  
                // delta_diff = -Mathf.Min( Mathf.Max( -(error_yrot_vertical.x * gains_yrot_vertical.x +  error_yrot_vertical.y *gains_yrot_vertical.y + error_yrot_vertical.z*gains_yrot_vertical.z), -5f), 5f);
            
        }

        // Set deflection of ailerons
        float deflection_left = -delta_left-delta_both+ delta_diff;
        float deflection_right = -delta_right-delta_both-delta_diff;

        // Set speed of propellers
        float RPM1 = RPM[0]-RPM_inner+RPM_left+RPM_all;
        float RPM2 = RPM[1]+RPM_inner+RPM_left+RPM_all;
        float RPM3 = RPM[2]+RPM_inner-RPM_left+RPM_all;
        float RPM4 = RPM[3]-RPM_inner-RPM_left+RPM_all;

        // Rotate moving parts
        pivot_aileron_right.transform.localRotation =  Quaternion.AngleAxis(+deflection_right, new Vector3(-p1.localPosition.x,p1.localPosition.y,p1.localPosition.z)- new Vector3(-p2.localPosition.x,p2.localPosition.y,p2.localPosition.z));
        pivot_aileron_left.transform.localRotation =  Quaternion.AngleAxis(-deflection_left, p1.localPosition-p2.localPosition);

        // pivot_aileron_left.transform.localRotation = Quaternion.Euler(deflection_left, 0, 0);
        // pivot_aileron_right.transform.localRotation = Quaternion.Euler(deflection_right, 0, 0);
        propellers[0].transform.Rotate(-Vector3.forward*RPM1/60*360*Time.deltaTime);   
        propellers[1].transform.Rotate(Vector3.forward*RPM2/60*360*Time.deltaTime);    
        propellers[2].transform.Rotate(-Vector3.forward*RPM3/60*360*Time.deltaTime);    
        propellers[3].transform.Rotate(Vector3.forward*RPM4/60*360*Time.deltaTime);   

         float q;
        if (flightMode == FlightMode.Horizontal){
            q = 0.5f * 1.225f * Mathf.Pow(linear_velocity.magnitude, 2);
        }else{
            q = 0.5f * 1.225f * Mathf.Pow((RPM1+RPM2+RPM3+RPM4)/2000,2);
        } 

        float CL = CL0-CL0 + CLalpha * aerodynamic_angle.x;
        // Define Aerodynamic forces magnitudes
        float lift_left = q * (0.003f*0+ 0.0178f*(-deflection_left)) * Sw;
        float lift_right = q * (0.003f*0+ 0.0178f*(-deflection_right)) * Sw;

        // float lift_wing = 1.0f/2.0f * 1.225f * (CL0 + aerodynamic_angle.x*CLalpha)        * Sw * Mathf.Pow(linear_velocity.magnitude,2);
        float lift_wing = q * CL ;
        float drag_wing = q * (CD0 + Mathf.Pow(aerodynamic_angle.x,2)*b);
        float stabilizer_lift =  q * (CLalpha * aerodynamic_angle.y) ;
        float stabilizer_drag =  q * (CD0 + Mathf.Pow(aerodynamic_angle.y,2)*b);

        // ---- UPDATE FORCES ----
        // Update wing lift and drag
        foreach (Force force in Left_Wing_Lift.Concat(Right_Wing_Lift)){
            force.update_force(Vector3.Normalize(Vector3.Cross(Vector3.forward, Quaternion.Inverse(aerodynamic_frame.rotation) * body_frame.rotation * Vector3.right)) * lift_wing * force.S);
        }
        foreach (Force force in Left_Wing_Drag.Concat(Right_Wing_Drag)){
            force.update_force(-Vector3.forward*drag_wing*force.S);
        }
        // Update vertical stabilizer lift and drag
        foreach (Force force in Up_Stabilizer_Lift.Concat(Down_Stabilizer_Lift)){
            force.update_force(Vector3.Normalize(Vector3.Cross(Vector3.forward, Quaternion.Inverse(aerodynamic_frame.rotation) * body_frame.rotation * Vector3.down)) * stabilizer_lift * force.S);
        }
        foreach (Force force in Up_Stabilizer_Drag.Concat(Down_Stabilizer_Drag)){
            force.update_force(-Vector3.forward*stabilizer_drag*force.S);
        }
        F_AL_Lift.update_force(Vector3.Normalize(Vector3.Cross(Vector3.forward, Quaternion.Inverse(aerodynamic_frame.rotation) * pivot_aileron_left.rotation * Vector3.right)) *lift_left);
        F_AR_Lift.update_force(Vector3.Normalize(Vector3.Cross(Vector3.forward, Quaternion.Inverse(aerodynamic_frame.rotation) * pivot_aileron_right.rotation * Vector3.right)) * lift_right);
        F_T1.update_force(Vector3.forward * RPM_to_thrust(RPM1));
        F_T2.update_force(Vector3.forward * RPM_to_thrust(RPM2));
        F_T3.update_force(Vector3.forward * RPM_to_thrust(RPM3));
        F_T4.update_force(Vector3.forward * RPM_to_thrust(RPM4));
        W.update_force(Vector3.down*g*mass);
     
        // Calculate total force in body frame
        F_total_body = Vector3.zero;
        foreach (Force F in forces){
            F_total_body += F.force;
        }

        // Calculate total force in inertial frame
        Vector3 F_total = Quaternion.Inverse(inertial_frame.rotation) * body_frame.rotation * F_total_body;
        linear_acceleration = F_total/mass;
        linear_velocity += linear_acceleration*Time.deltaTime;
        transform.position += linear_velocity*Time.deltaTime;

        Vector3 moment_v3 = Vector3.zero;
        foreach (Force force in forces){
            Vector3 difference = force.position - cg.transform.localPosition;
            moment_v3 += Vector3.Cross(new Vector3(difference.x, difference.y, 0), new Vector3(force.force.x, force.force.y, 0));
            moment_v3 += Vector3.Cross(new Vector3(difference.x, 0, difference.z), new Vector3(force.force.x, 0, force.force.z));
            moment_v3 += Vector3.Cross(new Vector3(0, difference.y, difference.z), new Vector3(0, force.force.y, force.force.z));
        }
      
        moment_v3.z += (RPM_to_torque(RPM1)-RPM_to_torque(RPM2)+RPM_to_torque(RPM3)-RPM_to_torque(RPM4));                      // Z Moment generated by torque of propellors (2 clockwise and 2 counterclockwise)
        moment = new double[3][] {new double[]{moment_v3.x},new double[]{moment_v3.y},new double[]{moment_v3.z}};

        double[][] coriolis = MatrixInverse.MatrixInverseProgram.MatrixProduct(A, angular_velocity);
        double[][] B = MatrixInverse.MatrixInverseProgram.CrossProduct(angular_velocity, coriolis);
        double[][] C = MatrixInverse.MatrixInverseProgram.MatrixSubstract(moment, B);
        angular_acceleration = MatrixInverse.MatrixInverseProgram.MatrixProduct(MatrixInverse.MatrixInverseProgram.MatrixInverse(A),C);
        angular_acceleration_v3 = new Vector3((float)angular_acceleration[0][0], (float)angular_acceleration[1][0], (float)angular_acceleration[2][0]);
        angular_velocity = MatrixInverse.MatrixInverseProgram.MatrixMultiply(angular_acceleration, Time.deltaTime);
        angular_velocity_v3 += new Vector3((float)angular_velocity[0][0], (float)angular_velocity[1][0], (float)angular_velocity[2][0]);
        transform.rotation = transform.rotation * Quaternion.Euler(angular_velocity_v3*Time.deltaTime) ;
        track_camera.transform.position = transform.position;

    }

    float RPM_to_thrust(float RPM){
        float thrust = Mathf.Pow((RPM/1237f), 1/0.479f);
        return thrust;
    }

    float RPM_to_torque(float RPM){
        float thrust = RPM_to_thrust(RPM);
        // Should be dot product of thrust vector and velocity vector
        float power = thrust*linear_velocity.magnitude;
        float torque = power / (RPM/60*360 /180* Mathf.PI);
        return torque;
    }

    float eulerToAngle(float euler){
        if (euler >= 180){
            return (euler-360);
        }else if(euler <= -180){
            return (euler +360);
        }else{
            return euler;
        }
    }

}
