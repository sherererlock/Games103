using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;                 // for collision


	Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f) * 0.1F;

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	float Signed_Distance_Function_Plane(Vector3 P, Vector3 N, Vector3 X)
    {
		return Vector3.Dot((X - P), N);
    }

    private Matrix4x4 Matrix_subtraction(Matrix4x4 a, Matrix4x4 b)
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                a[i, j] -= b[i, j];
            }
        }
        return a;
    }

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

		Vector3 position = Vector3.zero;
		int penetrate_count = 0;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		Matrix4x4 I = R * I_ref * R.transpose;

		for (int i = 0; i < vertices.Length; i ++)
        {
			Vector3 R_ri = R.MultiplyPoint(vertices[i]);
			Vector3 x_i = transform.position + R_ri;

			float distance = Signed_Distance_Function_Plane(P, N, x_i);
			if (distance >= 0f)
				continue;

            Vector3 v_i = v + Vector3.Cross(w, R_ri);
			if (Vector3.Dot(v_i, N) >= 0f)
				continue;

			Vector3 v_i_n = Vector3.Dot(v_i, N) * N;
			Vector3 v_i_t = v_i - v_i_n;

			float u_t = restitution; // 切线方向的反弹系数
			float u_n = restitution; // 法线方向的反弹系数

			float dynamic_friction = 1 - u_t * (1 + u_n) * v_i_n.magnitude / v_i_t.magnitude;
			float static_friction = 0;
			float a = Mathf.Max(dynamic_friction, static_friction);

			Vector3 v_i_n_new = -u_n * v_i_n;
			Vector3 v_i_t_new = a * v_i_t;

			Vector3 v_i_new = v_i_n_new + v_i_t_new;

			Matrix4x4 m_r_r_i = Get_Cross_Matrix(R_ri);

			float m_inverse = 1f / mass;
			Vector3 mass_scale = new Vector3(m_inverse, m_inverse, m_inverse);
			Matrix4x4 mass_scale_mat = Matrix4x4.Scale(mass_scale);

			Matrix4x4 K = Matrix_subtraction(mass_scale_mat, m_r_r_i * I.inverse * m_r_r_i);

			Vector3 j = K.inverse * (v_i_new - v_i);

			v = v + m_inverse * j;
			Vector3 dw = I.inverse * (Vector3.Cross(R_ri, j));
			w = w + dw;
		}
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (0, 0, 0);
			launched=true;
		}

		if(launched)
        {
            // Part I: Update velocities
            v += gravity * dt;
            v *= linear_decay;

			w *= angular_decay;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
            Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

            // Part III: Update position & orientation
            //Update linear status
            Vector3 x = transform.position;

            x += v * dt;

            //Update angular status
            Quaternion q = transform.rotation;

			Vector3 wt = 0.5f * w * dt;
			Quaternion temp = new Quaternion(wt.x, wt.y, wt.z, 0f);
			temp = temp * q;

			q.Set(q.x + temp.x, q.y + temp.y, q.z + temp.z, q.w + temp.w);

			// Part IV: Assign to the object
			transform.position = x;
            transform.rotation = q;
        }

	}
}
