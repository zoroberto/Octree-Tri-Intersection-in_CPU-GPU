using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using UnityEngine.Rendering;
using PBD;

public class MultiGPUPBD : MonoBehaviour
{
    public enum MyModel
    {
        IcoSphere_low,
        Torus,
        Bunny,
        Armadillo,
    };


    [Header("Deformable model")]
    public int number_object = 1;

    [Header("Random Range")]
    Vector3 rangeMin = new Vector3(-10f, 0f, 0f);
    Vector3 rangeMax = new Vector3(10f, 10f, 20f);


    [Header("3D model")]
    public MyModel model;
    [HideInInspector]
    private string modelName;

    [Header("Obj Parameters")]
    public float invMass = 1.0f;
    public float dt = 0.01f; // have to devide by 20
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    public int iteration = 5;
    public float convergence_factor = 1.5f;

    [Header("Distance Constrinat Parameters")]
    public float stretchStiffness = 1.0f;
    public float compressStiffness = 1.0f;
    [Header("Bending Constrinat Parameters")]
    public float bendingStiffness = 1.0f;
    [Header("Volume  Constrinat Parameters")]
    public float volumeStiffness = 1.0f;

    [Header("Collision")]
    public GameObject[] collidableObject;
    [Header("GPU Paramenter")]
    public ComputeShader computeShader;

    [Header("Rendering Paramenter")]
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]

    private Vector3[] Positions;
    private Vector3[] ProjectPositions;
    private Vector3[] WorldPositions;
    private Vector3[] Velocities;
    private Vector3[] Forces;
    private Vector3[] Normals;
    private List<Spring> distanceConstraints = new List<Spring>();
    private List<Triangle> triangles = new List<Triangle>();
    private List<Tetrahedron> tetrahedrons = new List<Tetrahedron>();
    private List<Bending> bendingConstraints = new List<Bending>();
    private int[] triArray;

    private int nodeCount;
    private int springCount;
    private int triCount; // size of triangle
    private int tetCount;
    private int bendingCount;


    [HideInInspector]
    GameObject[] deformableObjectList;

    private GPUPBD[] deformableGPUPBD;
    void SelectModelName()
    {
        switch (model)
        {
            case MyModel.IcoSphere_low: modelName = "icosphere_low.1"; break;
            case MyModel.Torus: modelName = "torus.1"; break;
            case MyModel.Bunny: modelName = "bunny.1"; break;
            case MyModel.Armadillo: modelName = "Armadillo.1"; break;
        }
    }



    void addDeformableObjectList()
    {
        deformableObjectList = new GameObject[number_object];
        HashSet<Vector3> generatedPositions = new HashSet<Vector3>();
        deformableGPUPBD = new GPUPBD[number_object];

        for (int i = 0; i < number_object; i++)
        {
            deformableObjectList[i] = new GameObject("Deformable Object " + i);
            deformableObjectList[i].transform.SetParent(this.transform);

            //set position of the object 1). randomize 2).set the coord
            Vector3 randomPosition;
            

            do
            {
                // Generate random position within the specified range
                float x = UnityEngine.Random.Range(rangeMin.x, rangeMax.x);
                float y = UnityEngine.Random.Range(rangeMin.y, rangeMax.y);
                float z = UnityEngine.Random.Range(rangeMin.z, rangeMax.z);

                randomPosition = new Vector3(x, y, z);
            } while (generatedPositions.Contains(randomPosition));

            deformableObjectList[i].transform.position = randomPosition;
            deformableObjectList[i].transform.localScale = transform.localScale;
            deformableObjectList[i].transform.rotation = transform.rotation;

            if (deformableObjectList[i] != null)
            {
                GPUPBD gpupbdScript = deformableObjectList[i].AddComponent<GPUPBD>();
                gpupbdScript.SetCoefficient(invMass, dt, gravity, iteration, stretchStiffness, compressStiffness, bendingStiffness, volumeStiffness);
                gpupbdScript.SetMeshData(modelName);
                ComputeShader tmpCS = Instantiate(computeShader);
               
                Shader tmp = Instantiate(renderingShader);
                Color randomColor = new Color(
                    UnityEngine.Random.value,
                    UnityEngine.Random.value,
                    UnityEngine.Random.value);
                //cpupbdScript.SetRenderer(tmp, matColor);
                gpupbdScript.SetRenderer(tmpCS,tmp, randomColor);
                gpupbdScript.SetCollidableObj(collidableObject);
                deformableGPUPBD[i] = gpupbdScript;
            }


        }
    }

    void Start()
    {
        //selecting tetgen model
        SelectModelName();

        //instantiate gameobject 
        addDeformableObjectList();
        //

    }



    private void OnGUI()
    {
        int w = Screen.width, h = Screen.height;
        GUIStyle style = new GUIStyle();
        Rect rect = new Rect(20, 40, w, h * 2 / 100);
        style.alignment = TextAnchor.UpperLeft;
        style.fontSize = h * 2 / 50;
        style.normal.textColor = Color.yellow;


        string text = string.Format("num. Obj :: " + number_object);
        GUI.Label(rect, text, style);

    }
}
