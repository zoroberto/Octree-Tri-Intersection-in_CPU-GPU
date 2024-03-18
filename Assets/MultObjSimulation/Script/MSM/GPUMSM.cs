using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using UnityEngine.Rendering;
using PBD;
using UnityEditor;

public class GPUMSM : MonoBehaviour
{
    private float dt = 0.005f; // have to devide by 20
    private Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    private int speed = 1;
    private GameObject[] collidableObjects;

    private int nodeCount;
    private int triCount;

    private string modelName;

    private bool useInteriorSpring = false;
    
    private int number_object;

    private Vector3[] Positions;
    private Vector3[] Velocities;
    private List<Triangle> triangles = new List<Triangle>();

    private List<int> initTrianglePtr = new List<int>();
    private List<Triangle> initTriangle = new List<Triangle>();

    private int[] triArray;
    private vertData[] vDataArray;
    private ComputeBuffer vertexBuffer;
    private ComputeBuffer triBuffer;

    private ComputeShader computeShaderObj;
    private Shader renderingShader;
    private Color matColor;
    private Material material;

    private ComputeBuffer positionsBuffer ;
    private ComputeBuffer velocitiesBuffer ;
    private ComputeBuffer triangleBuffer ;
    private ComputeBuffer triPtrBuffer;
    private ComputeBuffer triArrayBuffer;

    private ComputeBuffer floorBBBuffer ;
    private ComputeBuffer floorPositionsBuffer;
    private ComputeBuffer bbBoundingBuffer ;
    private ComputeBuffer floorCollisionResultBuffer ;
    private ComputeBuffer posTrianglesBuffer ;

    private int updatePosKernel;
    private int computenormalKernel;
    private int FindPosTrianglesKernel;

    private int findFloorMinMaxKernel;
    private int collisionWithFloorKernel;
    private int updateReverseVelocityKernel;

    private Bounding[] bbFloor;

    public void StartObj()
    {
        //if (collidableObjects != null) floorObj = collidableObjects[0];
        material = new Material(renderingShader);
        material.color = matColor;
        transformMesh();
        setupShader();
        setBuffData();
        setupComputeBuffer();
        setupComputeShader();
    }

    public int getNodeCount()
    {
        return nodeCount;
    }

    public int getTriCount()
    {
        return triCount;
    }

    public ComputeBuffer GetPositionBuffer()
    {
        return positionsBuffer;
    }
    public ComputeBuffer GetPosTrianglesBuffer()
    {
        return posTrianglesBuffer;
    }

    public void SetNumberObj(int numberObj)
    {
        number_object = numberObj;
    }

    public void SetCollidableObj(GameObject[] CollidableObjects)
    {
        collidableObjects = CollidableObjects;
    }
    public void SetCoefficient( float Dt, Vector3 Gravity,  int Speed)
    {
        dt = Dt;
        gravity = Gravity;
        speed = Speed;
    }
    public void SetMeshData(Vector3[] NodePositions, List<Triangle> MeshTriangle,
        int[] TriArray, List<Spring> Springs, List<Tetrahedron> Tetrahedrons)
    {
        Positions = NodePositions;
        triangles = MeshTriangle;
        triArray = TriArray;
    }

    public void SetMeshData(string TetGenModel, bool UseInteriorSpring)
    {
        modelName = TetGenModel;
        useInteriorSpring = UseInteriorSpring;
    }
    public void SetRenderer(ComputeShader computeShader, Shader RenderingShader, Color MatColor)
    {
        computeShaderObj = computeShader;
        renderingShader = RenderingShader;
        matColor = MatColor;
    }

    private void transformMesh()
    {
        string filePath = Application.dataPath + "/MultObjSimulation/TetModel/";
        LoadTetModel.LoadData(filePath + modelName, useInteriorSpring, this.gameObject);

        Positions = LoadTetModel.positions.ToArray();
        triangles = LoadTetModel.triangles;


        triArray = LoadTetModel.triangleArr.ToArray();
        //if (useTetVolumeConstraint)


        nodeCount = Positions.Length;
        triCount = triangles.Count;

        Velocities = new Vector3[nodeCount];
        Velocities.Initialize();

        vDataArray = new vertData[nodeCount];

        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i] = new vertData();
            vDataArray[i].pos = Positions[i];
            vDataArray[i].norms = Vector3.zero;
            vDataArray[i].uvs = Vector3.zero;
        }

        int triBuffStride = sizeof(int);
        triBuffer = new ComputeBuffer(triArray.Length,
            triBuffStride, ComputeBufferType.Default);


        int vertsBuffstride = 8 * sizeof(float);
        vertexBuffer = new ComputeBuffer(vDataArray.Length,
            vertsBuffstride, ComputeBufferType.Default);

        initTrianglePtr.Add(0);
        for (int i = 0; i < nodeCount; i++)
        {
            foreach (Triangle tri in triangles)
            {
                if (tri.vertices[0] == i || tri.vertices[1] == i || tri.vertices[2] == i)
                    initTriangle.Add(tri);
            }
            initTrianglePtr.Add(initTriangle.Count);
        }

        LoadTetModel.ClearData();

        // print("t count: " + initTrianglePtr.Count);
        // print("tri count: " + triangles.Count);
        // print("triArray: " + triArray.Length);
        //print("node count: " + nodeCount);
        //print("springCounnt: " + springCount);

    }

    private void setupShader()
    {
        material.SetBuffer(Shader.PropertyToID("vertsBuff"), vertexBuffer);
        material.SetBuffer(Shader.PropertyToID("triBuff"), triBuffer);
    }

    private void setBuffData()
    {

        vertexBuffer.SetData(vDataArray);
        triBuffer.SetData(triArray);

        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);
    }

    private void setupComputeBuffer()
    {
        positionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        positionsBuffer.SetData(Positions);

        velocitiesBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        velocitiesBuffer.SetData(Velocities);

        UInt3Struct[] forceUintArray = new UInt3Struct[nodeCount];
        forceUintArray.Initialize();


        List<MTriangle> initTriangle = new List<MTriangle>();
        List<int> initTrianglePtr = new List<int>();
        initTrianglePtr.Add(0);
        for (int i = 0; i < nodeCount; i++)
        {
            foreach (Triangle tri in triangles)
            {
                if (tri.vertices[0] == i || tri.vertices[1] == i || tri.vertices[2] == i)
                {
                    MTriangle tmpTri = new MTriangle();
                    tmpTri.v0 = tri.vertices[0];
                    tmpTri.v1 = tri.vertices[1];
                    tmpTri.v2 = tri.vertices[2];
                    initTriangle.Add(tmpTri);
                }
            }
            initTrianglePtr.Add(initTriangle.Count);
        }


        triangleBuffer = new ComputeBuffer(initTriangle.Count, (sizeof(int) * 3));
        triangleBuffer.SetData(initTriangle.ToArray());

        triPtrBuffer = new ComputeBuffer(initTrianglePtr.Count, sizeof(int));
        triPtrBuffer.SetData(initTrianglePtr.ToArray());

        triArrayBuffer = new ComputeBuffer(triArray.Length, sizeof(int));
        triArrayBuffer.SetData(triArray);

        Vector3[] _floorVertices = collidableObjects[0].GetComponent<MeshFilter>().mesh.vertices;
        List<Vector3> floorVertices = new List<Vector3>();


        for (int i = 0; i < _floorVertices.Length; i++)
        {
            floorVertices.Add(collidableObjects[0].transform.TransformPoint(_floorVertices[i]));
        }

        //if (bbBoundingBuffer != null) bbBoundingBuffer.Release();
        //if (floorCollisionResultBuffer != null) floorCollisionResultBuffer.Release();

        floorBBBuffer = new ComputeBuffer(1, sizeof(float) * 6);
        floorPositionsBuffer = new ComputeBuffer(_floorVertices.Length, sizeof(float) * 6);
        bbBoundingBuffer = new ComputeBuffer(1, sizeof(float) * 6);
        floorCollisionResultBuffer = new ComputeBuffer(1, sizeof(int));
        floorPositionsBuffer.SetData(floorVertices);

        bbBoundingBuffer = new ComputeBuffer(1, sizeof(float) * 6);
        floorCollisionResultBuffer = new ComputeBuffer(1, sizeof(int));
        posTrianglesBuffer = new ComputeBuffer(triCount, sizeof(float) * 9);

    }

    private void setupComputeShader()
    {
        updatePosKernel = computeShaderObj.FindKernel("UpdatePosKernel");
        computenormalKernel = computeShaderObj.FindKernel("computenormalKernel");
        FindPosTrianglesKernel = computeShaderObj.FindKernel("FindPosTriangles");

        findFloorMinMaxKernel = computeShaderObj.FindKernel("FindFloorMinMax");
        collisionWithFloorKernel = computeShaderObj.FindKernel("CollisionWithFloor");
        updateReverseVelocityKernel = computeShaderObj.FindKernel("UpdateReverseVelocity");

        computeShaderObj.SetInt("nodeCount", nodeCount);
        computeShaderObj.SetInt("triCount", triCount);
        computeShaderObj.SetInt("numberObj", number_object);
        computeShaderObj.SetFloat("dt", dt);
       
        computeShaderObj.SetBuffer(updatePosKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(updatePosKernel, "Velocities", velocitiesBuffer);
        computeShaderObj.SetBuffer(updatePosKernel, "vertsBuff", vertexBuffer); //passing to rendering


        computeShaderObj.SetBuffer(computenormalKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(computenormalKernel, "Triangles", triangleBuffer);
        computeShaderObj.SetBuffer(computenormalKernel, "TrianglePtr", triPtrBuffer);
        computeShaderObj.SetBuffer(computenormalKernel, "vertsBuff", vertexBuffer); //passing to rendering

        computeShaderObj.SetBuffer(FindPosTrianglesKernel, "posTriangles", posTrianglesBuffer);
        computeShaderObj.SetBuffer(FindPosTrianglesKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(FindPosTrianglesKernel, "triArray", triArrayBuffer);

        // FindFloorMinMax
        computeShaderObj.SetBuffer(findFloorMinMaxKernel, "floorPositions", floorPositionsBuffer);
        computeShaderObj.SetBuffer(findFloorMinMaxKernel, "floorBB", floorBBBuffer);
        computeShaderObj.Dispatch(findFloorMinMaxKernel, 1, 1, 1);

        bbFloor = new Bounding[1];
        floorBBBuffer.GetData(bbFloor);

        computeShaderObj.SetBuffer(collisionWithFloorKernel, "floorCollisionResult", floorCollisionResultBuffer);
        computeShaderObj.SetBuffer(collisionWithFloorKernel, "floorBB", floorBBBuffer);
        computeShaderObj.SetBuffer(collisionWithFloorKernel, "bbBounding", bbBoundingBuffer);
        computeShaderObj.SetBuffer(collisionWithFloorKernel, "Positions", positionsBuffer);

        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "Velocities", velocitiesBuffer);
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "floorCollisionResult", floorCollisionResultBuffer);
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "vertsBuff", vertexBuffer); //passing to rendering
    }

    public void UpdateObj()
    {

        dispatchComputeShader();
        //setData
        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
        material.SetPass(0);
        Graphics.DrawProcedural(material, bounds, MeshTopology.Triangles, triArray.Length,
            1, null, null, ShadowCastingMode.On, true, gameObject.layer);
        

    }

    void dispatchComputeShader()
    {
        //for (int i = 0; i < speed; i++)
        {

            computeShaderObj.Dispatch(updatePosKernel, Mathf.CeilToInt(nodeCount / 1024.0f), 1, 1);
        }

        computeShaderObj.Dispatch(computenormalKernel, Mathf.CeilToInt(nodeCount / 1024.0f), 1, 1);
        computeShaderObj.Dispatch(FindPosTrianglesKernel, Mathf.CeilToInt(triCount / 1024.0f), 1, 1);


        computeShaderObj.Dispatch(collisionWithFloorKernel, 1, 1, 1);
        computeShaderObj.Dispatch(updateReverseVelocityKernel, Mathf.CeilToInt(nodeCount / 1024f), 1, 1);

    }

   
    private void OnDestroy()
    {
        if (enabled)
        {

            vertexBuffer.Dispose();
            triBuffer.Dispose();
            positionsBuffer.Dispose();
            posTrianglesBuffer.Dispose();
            velocitiesBuffer.Dispose();
            triangleBuffer.Dispose();
            triPtrBuffer.Dispose();

            bbBoundingBuffer.Dispose();
            floorCollisionResultBuffer.Dispose();
            floorBBBuffer.Dispose();
            floorPositionsBuffer.Dispose();

        }

    }
}
