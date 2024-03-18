using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using UnityEngine.Rendering;
using PBD;
using Octree;
using ExporterImporter;
using UnityEngine.UIElements;

public class MultiGPUMSM : MonoBehaviour
{
    public enum MyModel
    {
        IcoSphere_low,
        Torus,
        Bunny,
        Armadillo,
        Cube
    };


    [Header("Deformable model")]
    public int number_object = 1;
    [Header("Random Range")]
    Vector3 rangeMin = new Vector3(-10f, 0f, 0f);
    Vector3 rangeMax = new Vector3(10f, 10f, 20f);

    [Header("3D model")]
    public MyModel model;

    [Header("Import CSV")]
    public string csv_file = "object_positions.csv";

    [HideInInspector]
    private string modelName;

    [Header("Obj Parameters")]
    public float dt = 0.005f; // have to devide by 20
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);

    public int speed = 1;

    [Header("Geometric Parameters")]
    public bool useInteriorSpring = false;


    [Header("Collision")]
    public GameObject[] collidableObject;

    [Header("GPU Paramenter")]
    public ComputeShader computeShader;
    public ComputeShader OctreeAabbCS;

    [Header("Rendering")]
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private GameObject[] deformableObjectList;
    private GPUMSM[] deformableGPUMSM;

    GPUMSM gpumsmScript;

    [Header("Debug Mode")]
    public bool debugModeData = true;
    public bool debugModeL0 = true;
    public bool debugModeL1 = true;
    public bool debugModeL2 = true;
    public bool debugModeTriangle = true;
   

    private Vector2Int[] indicies;
    private int nodeCount;
    private int triCount;

    private readonly int octree_size = 73;

    // kernel IDs
    private int bindPositionsKernel;
    private int bindPosTrianglesKernel;
    private int findBBMinMaxKernel;
    private int implementOctreeKernel;
    private int checkCollisionL0Kernel;
    private int checkCollisionL1Kernel;
    private int checkCollisionL2Kernel;
    private int TriIntersectionKernel;
    private int RemoveTriKernel;

    // Compute Buffer
    private ComputeBuffer positionsBuffer;
    private ComputeBuffer globalPositionsBuffer;
    private ComputeBuffer posTrianglesBuffer;
    private ComputeBuffer globalPosTrianglesBuffer;
    private ComputeBuffer bbMinMaxBuffer;
    private ComputeBuffer rangeObjIndexBuffer;
    private ComputeBuffer bbOctreeBuffer;
    private ComputeBuffer pairIndexL0Buffer;
    private ComputeBuffer pairIndexL1Buffer;
    private ComputeBuffer pairIndexL2Buffer;
    private ComputeBuffer collisionResultsL0Buffer;
    private ComputeBuffer collisionResultsL1Buffer;
    private ComputeBuffer collisionResultsL2Buffer;
    private ComputeBuffer collisionResultTri1Buffer;
    private ComputeBuffer collisionResultTri2Buffer;
    private ComputeBuffer collisionResultTriBuffer;
    // data
    private OctreeData[] bbOctree;
    
    private List<PairData> pairIndexL0 = new List<PairData>();
    private List<PairData> pairIndexL1 = new List<PairData>();
    private List<PairData> pairIndexL2 = new List<PairData>();
   
    private struct Tri
    {
        public Vector3 vertex0, vertex1, vertex2;
    }
    private Tri[] posTriangles;

    // collision pair
    private List<int> collidablePairIndexL0 = new List<int>();
    private List<int> collidablePairIndexL1 = new List<int>();
    private List<int> collidablePairIndexL2 = new List<int>();
    
    private int[] collisionresultsL0;
    private int[] collisionresultsL1;
    private int[] collisionresultsL2;
    private int[] collisionresultsTri1;
    private int[] collisionresultsTri2;
    private int[] collisionresultsTri;

    void Start()
    {
        
        SelectModelName();
        addDeformableObjectList();
        AddOctreePairIndex();

        FindKernelIDs();
        SetupComputeBuffer();
        SetupComputeShader();
    }

    void SelectModelName()
    {
        switch (model)
        {
            case MyModel.IcoSphere_low: modelName = "icosphere_low.1"; break;
            case MyModel.Torus: modelName = "torus.1"; break;
            case MyModel.Bunny: modelName = "bunny.1"; break;
            case MyModel.Armadillo: modelName = "Armadillo.1"; break;
            case MyModel.Cube: modelName = "33cube.1"; break;
        }

        
    }
    void addDeformableObjectList()
    {
        deformableObjectList = new GameObject[number_object];
        HashSet<Vector3> generatedPositions = new HashSet<Vector3>();
        deformableGPUMSM = new GPUMSM[number_object];
        indicies = new Vector2Int[number_object];
        int st_index = 0;

        List<List<string>> csvData = ExporterAndImporter.ReadCSVFile(csv_file);
        Vector3 position = new Vector3();

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
            //deformableObjectList[i].transform.position = randomPosition;

            List<string> row = csvData[i];
            for (int j = 1; j < row.Count; j++)
            {
                float x = float.Parse(row[0]);
                float y = float.Parse(row[1]);
                float z = float.Parse(row[2]);

                position = new Vector3(x, y, z);
            }
            deformableObjectList[i].transform.position = position;

            
            deformableObjectList[i].transform.localScale = transform.localScale;
            deformableObjectList[i].transform.rotation = transform.rotation;

            if (deformableObjectList[i] != null)
            {
                gpumsmScript = deformableObjectList[i].AddComponent<GPUMSM>();
                gpumsmScript.SetCoefficient( dt, gravity, speed);
                gpumsmScript.SetMeshData(modelName, useInteriorSpring);
                Shader tmp = Instantiate(renderingShader);
                ComputeShader tmpCS = Instantiate(computeShader);
                Color randomColor = new Color(
                    UnityEngine.Random.value,
                    UnityEngine.Random.value,
                    UnityEngine.Random.value);
                //gpumsmScript.SetRenderer(tmp, matColor);
                gpumsmScript.SetRenderer(tmpCS, tmp, randomColor);
                gpumsmScript.SetCollidableObj(collidableObject);
                deformableGPUMSM[i] = gpumsmScript;


                deformableGPUMSM[i].StartObj();

                nodeCount = deformableGPUMSM[i].getNodeCount();
                triCount = deformableGPUMSM[i].getTriCount();

                indicies[i] = new Vector2Int(st_index, st_index + nodeCount);
                st_index += nodeCount;
            }

        }
    }

    int calculateStartIndex(int object_index, int level)
    {
        if (level == 1) return object_index * 73 + 1;
        if (level == 2) return object_index * 73 + 9;
        return object_index * 73;
    }

    int calculateEndIndex(int object_index, int level)
    {
        if (level == 1) return object_index * 73 + 8;
        if (level == 2) return object_index * 73 + 73;
        return object_index * 73;
    }

    private void AddOctreePairIndex()
    {
        for (int i = 0; i < number_object; i++)
        {
            var l0_st_idx = calculateStartIndex(i, 0);
            var l1_st_idx = calculateStartIndex(i, 1);
            var l2_st_idx = calculateStartIndex(i, 2);

            var l0_end_idx = calculateEndIndex(i, 0);
            var l1_end_idx = calculateEndIndex(i, 1);
            var l2_end_idx = calculateEndIndex(i, 2);

            for (int j = 0; j < number_object; j++)
            {
                if (i == j) break;

                var j_l0_st_idx = calculateStartIndex(j, 0);
                var j_l0_end_idx = calculateStartIndex(j, 0);

                var j_l1_st_idx = calculateStartIndex(j, 1);
                var j_l2_st_idx = calculateStartIndex(j, 2);

                var j_l1_end_idx = calculateEndIndex(j, 1);
                var j_l2_end_idx = calculateEndIndex(j, 2);


                for (int n = l0_st_idx; n <= l0_end_idx; n++)
                {
                    for (int m = j_l0_st_idx; m <= j_l0_end_idx; m++)
                    {
                        pairIndexL0.Add(new PairData
                        {
                            i1 = m,
                            i2 = n
                        });
                    }
                }
                for (int n = l1_st_idx; n <= l1_end_idx; n++)
                {
                    for (int m = j_l1_st_idx; m <= j_l1_end_idx; m++)
                    {
                        pairIndexL1.Add(new PairData
                        {
                            i1 = m,
                            i2 = n
                        });
                    }
                }
                for (int n = l2_st_idx; n < l2_end_idx; n++)
                {
                    for (int m = j_l2_st_idx; m < j_l2_end_idx; m++)
                    {
                       
                        pairIndexL2.Add(new PairData
                        {
                            i1 = m,
                            i2 = n
                        });
                    }
                }

                
            }
        }

       
    }

    private void FindKernelIDs()
    {
        bindPositionsKernel = OctreeAabbCS.FindKernel("BindGlobalPositions");
        bindPosTrianglesKernel = OctreeAabbCS.FindKernel("BindGlobalPosTriangles");
        findBBMinMaxKernel = OctreeAabbCS.FindKernel("FindBBMinMax");

        implementOctreeKernel = OctreeAabbCS.FindKernel("ImplementOctree");
        checkCollisionL0Kernel = OctreeAabbCS.FindKernel("CheckCollisionL0");
        checkCollisionL1Kernel = OctreeAabbCS.FindKernel("CheckCollisionL1");
        checkCollisionL2Kernel = OctreeAabbCS.FindKernel("CheckCollisionL2");
        TriIntersectionKernel = OctreeAabbCS.FindKernel("TriIntersection");
        RemoveTriKernel = OctreeAabbCS.FindKernel("RemoveTriKernel");

    }

    private void SetupComputeBuffer()
    {
        
       
        collisionresultsL0 = new int[pairIndexL0.Count];
        collisionresultsL1 = new int[pairIndexL1.Count];
        collisionresultsL2 = new int[pairIndexL2.Count];
        collisionresultsTri1 = new int[number_object * triCount];
        collisionresultsTri2 = new int[number_object * triCount];
        collisionresultsTri = new int[number_object * triCount];

        globalPositionsBuffer = new ComputeBuffer(nodeCount * number_object, sizeof(float) * 3);
        positionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        posTrianglesBuffer = new ComputeBuffer(triCount, sizeof(float) * 9);
        globalPosTrianglesBuffer = new ComputeBuffer(triCount * number_object, sizeof(float) * 9);
        
        bbMinMaxBuffer = new ComputeBuffer(number_object, sizeof(float) * 6);

        rangeObjIndexBuffer = new ComputeBuffer(number_object, sizeof(int) * 2);
        bbOctreeBuffer = new ComputeBuffer(number_object * octree_size, sizeof(float) * 13);

        pairIndexL0Buffer = new ComputeBuffer(pairIndexL0.Count, sizeof(int) * 2);
        pairIndexL1Buffer = new ComputeBuffer(pairIndexL1.Count, sizeof(int) * 2);
        pairIndexL2Buffer = new ComputeBuffer(pairIndexL2.Count, sizeof(int) * 2);

        collisionResultsL0Buffer = new ComputeBuffer(pairIndexL0.Count, sizeof(int));
        collisionResultsL1Buffer = new ComputeBuffer(pairIndexL1.Count, sizeof(int));
        collisionResultsL2Buffer = new ComputeBuffer(pairIndexL2.Count, sizeof(int));
        collisionResultTri1Buffer = new ComputeBuffer(number_object * triCount, sizeof(int));
        collisionResultTri2Buffer = new ComputeBuffer(number_object * triCount, sizeof(int));
        collisionResultTriBuffer = new ComputeBuffer(number_object * triCount, sizeof(int) * 2);
    }

    private void SetupComputeShader()
    {
        bbOctree = new OctreeData[number_object * octree_size];
        posTriangles = new Tri[number_object * triCount];

        OctreeAabbCS.SetInt("numberObj", number_object);
        OctreeAabbCS.SetInt("nodeCount", nodeCount);
        OctreeAabbCS.SetInt("triCount", triCount);

        rangeObjIndexBuffer.SetData(indicies);
        pairIndexL0Buffer.SetData(pairIndexL0);
        pairIndexL1Buffer.SetData(pairIndexL1);
        pairIndexL2Buffer.SetData(pairIndexL2);
        collisionresultsTri1.Initialize();
        collisionresultsTri2.Initialize();

        Tri[] globalPos =  new Tri[number_object * triCount];
        globalPos.Initialize();
        globalPosTrianglesBuffer.SetData(globalPos);
        collisionResultTri1Buffer.SetData(collisionresultsTri1);
        collisionResultTri2Buffer.SetData(collisionresultsTri2);
        collisionResultTriBuffer.SetData(collisionresultsTri);

        // BindGlobalPositions
        OctreeAabbCS.SetBuffer(bindPositionsKernel, "globalPositions", globalPositionsBuffer);

        // BindGlobalPosTriangles
        OctreeAabbCS.SetBuffer(bindPosTrianglesKernel, "globalPosTriangles", globalPosTrianglesBuffer);

        // findBBMinMaxKernel
        OctreeAabbCS.SetBuffer(findBBMinMaxKernel, "bbMinMax", bbMinMaxBuffer);
        OctreeAabbCS.SetBuffer(findBBMinMaxKernel, "rangeObjIndex", rangeObjIndexBuffer);
        OctreeAabbCS.SetBuffer(findBBMinMaxKernel, "globalPositions", globalPositionsBuffer);

        // ImplementOctree
        OctreeAabbCS.SetBuffer(implementOctreeKernel, "bbMinMax", bbMinMaxBuffer);
        OctreeAabbCS.SetBuffer(implementOctreeKernel, "bbOctree", bbOctreeBuffer);

        // CheckCollisionLv0
        OctreeAabbCS.SetBuffer(checkCollisionL0Kernel, "bbOctree", bbOctreeBuffer); 
        OctreeAabbCS.SetBuffer(checkCollisionL0Kernel, "collisionResultL0", collisionResultsL0Buffer);
        OctreeAabbCS.SetBuffer(checkCollisionL0Kernel, "pairIndexL0", pairIndexL0Buffer);
        // CheckCollisionL1
        OctreeAabbCS.SetBuffer(checkCollisionL1Kernel, "pairIndexL1", pairIndexL1Buffer);
        OctreeAabbCS.SetBuffer(checkCollisionL1Kernel, "collisionResultL1", collisionResultsL1Buffer);
        OctreeAabbCS.SetBuffer(checkCollisionL1Kernel, "bbOctree", bbOctreeBuffer);

        // CheckCollisionL2
        OctreeAabbCS.SetBuffer(checkCollisionL2Kernel, "pairIndexL2", pairIndexL2Buffer);
        OctreeAabbCS.SetBuffer(checkCollisionL2Kernel, "collisionResultL2", collisionResultsL2Buffer);
        OctreeAabbCS.SetBuffer(checkCollisionL2Kernel, "bbOctree", bbOctreeBuffer);

        OctreeAabbCS.SetBuffer(RemoveTriKernel, "collisionResultTri1", collisionResultTri1Buffer);
        OctreeAabbCS.SetBuffer(RemoveTriKernel, "collisionResultTri2", collisionResultTri2Buffer);

        // TriIntersection
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "globalPosTriangles", globalPosTrianglesBuffer); 
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "collisionResultTri1", collisionResultTri1Buffer); 
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "collisionResultTri2", collisionResultTri2Buffer); 
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "collisionResultTri", collisionResultTriBuffer); 
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "pairIndexL0", pairIndexL0Buffer);
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "bbOctree", bbOctreeBuffer);
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "collisionResultL0", collisionResultsL0Buffer);
        
    }


    private void Update()
    {
        
        for (int i = 0; i < number_object; i++)
        {
            OctreeAabbCS.SetInt("objectIndex", i);
            deformableGPUMSM[i].UpdateObj();

            //if (deformableGPUMSM[i].GetPositionBuffer() != null) positionsBuffer.Release();
            positionsBuffer = deformableGPUMSM[i].GetPositionBuffer();
            posTrianglesBuffer = deformableGPUMSM[i].GetPosTrianglesBuffer();
            
           

            //if (deformableGPUMSM[i].GetPositionBuffer() != null) positionsBuffer.Release();
            OctreeAabbCS.SetBuffer(bindPositionsKernel, "positions", positionsBuffer);
            OctreeAabbCS.SetBuffer(bindPosTrianglesKernel, "posTriangles", posTrianglesBuffer);


            int numGroups_Pos = Mathf.CeilToInt(nodeCount / 1024f);
            OctreeAabbCS.Dispatch(bindPositionsKernel, numGroups_Pos, 1, 1);

            int numGroups_Tri = Mathf.CeilToInt(triCount * number_object / 1024f);
            OctreeAabbCS.Dispatch(bindPosTrianglesKernel, numGroups_Tri, numGroups_Tri, 1);

        }

        DispatchComputeShader();


        GetDataToCPU();
    }

    private void DispatchComputeShader()
    {


        OctreeAabbCS.Dispatch(findBBMinMaxKernel, Mathf.CeilToInt(number_object / 1024f), 1, 1);
        OctreeAabbCS.Dispatch(implementOctreeKernel, Mathf.CeilToInt(number_object / 1024f), 1, 1);
        OctreeAabbCS.Dispatch(checkCollisionL0Kernel, Mathf.CeilToInt(pairIndexL0.Count/ 1024f), 1, 1);
        OctreeAabbCS.Dispatch(checkCollisionL1Kernel, Mathf.CeilToInt(pairIndexL1.Count / 1024f), 1, 1);
        OctreeAabbCS.Dispatch(checkCollisionL2Kernel, Mathf.CeilToInt(pairIndexL2.Count / 1024f), 1, 1);
        
        OctreeAabbCS.Dispatch(RemoveTriKernel, Mathf.CeilToInt(triCount * number_object / 1024f), 1, 1);
        int numGroups_Tri =  Mathf.CeilToInt(triCount * number_object / 32);
        OctreeAabbCS.Dispatch(TriIntersectionKernel, numGroups_Tri, numGroups_Tri, 1);       
    }

    private void GetDataToCPU()
    {
        if (debugModeData)
        bbOctreeBuffer.GetData(bbOctree);


        collidablePairIndexL0.Clear();
        collidablePairIndexL1.Clear();
        collidablePairIndexL2.Clear();


        if (debugModeL0)
        {
            collisionResultsL0Buffer.GetData(collisionresultsL0);

            for (int i = 0; i < collisionresultsL0.Length; i++)
            {
                if (collisionresultsL0[i] == 1)
                {
                    //print("i " + i);
                    collidablePairIndexL0.Add(i);

                }
            }
        }

        if (debugModeL1)
        {
            collisionResultsL1Buffer.GetData(collisionresultsL1);


            for (int i = 0; i < collisionresultsL1.Length; i++)
            {
                //print($"Lv1 i {i} {collisionresultsL1[i]}");
                if (collisionresultsL1[i] == 1)
                {
                    collidablePairIndexL1.Add(i);
                }
            }
        }

        if (debugModeL2)
        {
            collisionResultsL2Buffer.GetData(collisionresultsL2);

            for (int i = 0; i < collisionresultsL2.Length; i++)
            {
                //print($"Lv1 i {i} {collisionresultsL2[i]}");
                if (collisionresultsL2[i] == 1)
                {
                    collidablePairIndexL2.Add(i);

                }
            }
        }
        

        //collisionPairBuffer.GetData(collisionPairResults);
        // print($"i {collisionPairResults.Length}");

        //for (int i = 0; i < collisionPairResults.Length; i++)
        //{
            //print($"coll i {i} {collisionPairResults[i]}");

        //}
        
        // print(posTriangles.Length);
        if(debugModeTriangle)
        {
            collisionResultTri1Buffer.GetData(collisionresultsTri1);
            collisionResultTri2Buffer.GetData(collisionresultsTri2);
           
            globalPosTrianglesBuffer.GetData(posTriangles);
        }

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


    private void OnDrawGizmos()
    {
        if (bbOctree != null && debugModeData)
        {
            for (int i = 0; i < bbOctree.Length; i++)
            {
                Gizmos.color = Color.red;
                for (int p = 0; p < collidablePairIndexL0.Count; p++)
                {
                    if (i == pairIndexL0[collidablePairIndexL0[p]].i1)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                    }

                    if (i == pairIndexL0[collidablePairIndexL0[p]].i2)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                    }

                }
                
                Gizmos.color = Color.green;
                for (int p = 0; p < collidablePairIndexL1.Count; p++)
                {
                    if (i == pairIndexL1[collidablePairIndexL1[p]].i1)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);

                    }

                    if (i == pairIndexL1[collidablePairIndexL1[p]].i2)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);

                    }
                }
                
                

                Gizmos.color = Color.blue;
                for (int p = 0; p < collidablePairIndexL2.Count; p++)
                {
                    if (i == pairIndexL2[collidablePairIndexL2[p]].i1)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);

                    }

                    if (i == pairIndexL2[collidablePairIndexL2[p]].i2)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);

                    }
                }
            } 
        }

        if(debugModeTriangle)
        if(posTriangles != null)
        {
            for (int c = 0; c < collisionresultsTri1.Length; c++)
            {
                if (collisionresultsTri1[c] == 1)  DrawTriangle(posTriangles[c], Color.green);                
            }

            for (int c = 0; c < collisionresultsTri2.Length; c++)
            {
                if (collisionresultsTri2[c] == 1)  DrawTriangle(posTriangles[c], Color.green);
            }
           
        }
    }

    private void DrawTriangle(Tri triangle, Color color)
    {
        Gizmos.color = color;
        Gizmos.DrawLine(triangle.vertex0, triangle.vertex1);
        Gizmos.DrawLine(triangle.vertex1, triangle.vertex2);
        Gizmos.DrawLine(triangle.vertex2, triangle.vertex0);
    }


    private void OnDestroy()
    {
        if (enabled)
        {
            positionsBuffer.Release();
            globalPositionsBuffer.Release();
            bbMinMaxBuffer.Release();
            rangeObjIndexBuffer.Release();
            bbOctreeBuffer.Release();
            pairIndexL0Buffer.Release();
            pairIndexL1Buffer.Release();
            pairIndexL2Buffer.Release();
            collisionResultsL0Buffer.Release();
            collisionResultsL1Buffer.Release();
            collisionResultsL2Buffer.Release();
        }
    }
}
