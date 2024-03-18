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
using UnityEngine.Video;

public class MultiCPUMSM : MonoBehaviour
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

    [Header("Rendering")]
    public Shader renderingShader;
    public Color matColor;

    [Header("Debug Mode")]
    public bool debugModeLv0 = true; 
    public bool debugModeLv1 = true; 
    public bool debugModeLv2 = true; 

    [HideInInspector]
    private GameObject[] deformableObjectList;
    private CPUMSM[] deformableCPUMSM;
    // octree
    private OctreeData[] custOctree;
    private List<PairData> pairIndexL0 = new List<PairData>();
    private List<PairData> pairIndexL1 = new List<PairData>();
    private List<PairData> pairIndexL2 = new List<PairData>();
    private readonly int octree_size = 73;
    private OctreeData[] custBB;
    
    // collidable pair
    private readonly List<int> collidablePairIndexL0 = new List<int>();
    private readonly List<PairData> collidablePairIndexL1 = new List<PairData>();
    private readonly List<PairData> collidablePairIndexL2 = new List<PairData>();
    private readonly List<Vector2Int> collidableTriIndex = new List<Vector2Int>();

    private int triCount;

    public class Tris
    {
        public Vector3 vertex0, vertex1, vertex2; // �ﰢ���� ������
        public Vector3 p_vertex0, p_vertex1, p_vertex2; // �ﰢ���� ���� ��ġ
        public Vector3 vel0, vel1, vel2;

        public Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
        public float deltaTime = 0.01f;

        public void update()
        {
            this.vel0 += this.gravity * this.deltaTime;
            this.vertex0 += this.vel0 * this.deltaTime;

            this.vel1 += this.gravity * this.deltaTime;
            this.vertex1 += this.vel1 * this.deltaTime;

            this.vel2 += this.gravity * this.deltaTime;
            this.vertex2 += this.vel2 * this.deltaTime;
        }

        public void setZeroGravity()
        {
            this.gravity = Vector3.zero;
        }

        public void setInverseGravity()
        {
            this.gravity *= -1.0f;
        }

        public Vector3 getAverageVelocity()
        {
            return (this.vel0 + this.vel1 + this.vel2) / 3.0f;
        }
    }

     public class Line
    {
        public Vector3 p0, p1; // �ﰢ���� ������

        public Vector3 direction
        {
            get { return (p1 - p0).normalized; }
        }

        public Vector3 origin
        {
            get { return p0; }
        }
    }

    private List<Tris> posTriangle;

    // public struct Tri
    // {
    //     public Vector3 vertex0, vertex1, vertex2;
    // }
    // private Tri[] posTriangles;

    Vector3 hitPoint = new Vector3();
    float separationDistance = 0.05f;

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

    void Awake()
    {
        SelectModelName();
        addDeformableObjectList();
        AddOctreePairIndex();
    }

    void addDeformableObjectList()
    {
        deformableObjectList = new GameObject[number_object];
        deformableCPUMSM = new CPUMSM[number_object];
        custOctree = new OctreeData[number_object * octree_size];

        List<List<string>> csvData = ExporterAndImporter.ReadCSVFile(csv_file);
        Vector3 position = new Vector3();

        for (int i = 0; i < number_object; i++)
        {
            deformableObjectList[i] = new GameObject("Deformable Object " + i);
            deformableObjectList[i].transform.SetParent(transform);

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
                CPUMSM cpumsmScript = deformableObjectList[i].AddComponent<CPUMSM>();

                cpumsmScript.SetCoefficient(dt, gravity, speed);
                cpumsmScript.SetMeshData(modelName, useInteriorSpring);

                Shader tmp = Instantiate(renderingShader);
                Color randomColor = new Color(
                    UnityEngine.Random.value,
                    UnityEngine.Random.value,
                    UnityEngine.Random.value);

                //cpumsmScript.SetRenderer(tmp, matColor);
                cpumsmScript.SetRenderer(tmp, randomColor);
                cpumsmScript.SetCollidableObj(collidableObject);

                deformableCPUMSM[i] = cpumsmScript;
                deformableCPUMSM[i].StartObj();

                triCount = deformableCPUMSM[i].getTriCount();
            }

            //posTriangles = new Tri[triCount * number_object];
            posTriangle = new List<Tris>();
        }
    }

    int calculateStartIndex(int object_index, int level)
    {
        if (level == 1) return object_index * 73 + 1;
        if (level == 2) return object_index * 73 + 9;
        return object_index * 73 ;
    }

    int calculateEndIndex(int object_index, int level)
    {
        if (level == 1) return object_index * 73 + 8;
        if (level == 2) return object_index * 73 + 73;
        return object_index * 73 ;
    }

    private void AddOctreePairIndex()
    {
        pairIndexL0.Clear();
        pairIndexL1.Clear();
        pairIndexL2.Clear();
        for(int i = 0; i < number_object; i++)
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


                for (int n = l0_st_idx; n<=l0_end_idx; n++)
                {
                    for (int m = j_l0_st_idx; m <= j_l0_end_idx; m++)
                    {
                        pairIndexL0.Add( new PairData
                        {
                            i1 = n,
                            i2 = m
                        });
                        
                    }
                }
                for (int n = l1_st_idx; n <= l1_end_idx; n++)
                {
                    for (int m = j_l1_st_idx; m <= j_l1_end_idx; m++)
                    {
                        pairIndexL1.Add(new PairData
                        {
                            i1 = n,
                            i2 = m,
                        });
                    }
                }
                for (int n = l2_st_idx; n < l2_end_idx; n++)
                {
                    for (int m = j_l2_st_idx; m < j_l2_end_idx; m++)
                    {
                      
                        pairIndexL2.Add(new PairData
                        {
                            i1 = n,
                            i2 = m,
                        });
                    }
                }
            }
        }
    }

    
    void UpdatePosition()
    {
        for (int i = 0; i < posTriangle.Count; i++)
        {
            posTriangle[i].update();
        }
    }

    private void Update()
    {
        custBB = new OctreeData[number_object];
        posTriangle.Clear();
        for (int i = 0; i < number_object; i++)
        {
            ImplementOctree(i);
            AddTriangles(i);
        }
        
        // CheckCollisionOctreeLv0();
        // CheckCollisionOctreeLv1();
        // CheckCollisionOctreeLv2();

        CheckTriIntersection();
    }

    private void ImplementOctree(int i) 
    {
        // OctreeData[] custBB = new OctreeData[number_object];
        // for (int i = 0; i < number_object; i++)
        {
            deformableCPUMSM[i].UpdateObj();
            Vector3[] position = deformableCPUMSM[i].GetPosition();

            CustMesh.Getvertices(position);
            custBB[i].min = CustMesh.minPos;
            custBB[i].max = CustMesh.maxPos;
            custBB[i].center = (custBB[i].max + custBB[i].min) / 2;
            custBB[i].size = custBB[i].max - custBB[i].min;

            deformableCPUMSM[i].SetBoundingBox(custBB[i].min, custBB[i].max);

            // Lv0, Initialize Lv0
            custOctree[i * octree_size].center = custBB[i].center; // center Lv0
            custOctree[i * octree_size].size = custBB[i].size; // size Lv0
            custOctree[i * octree_size].min = custBB[i].min; // min value Lv0
            custOctree[i * octree_size].max = custBB[i].max; // max value Lv0

            Vector3 centerOct = custOctree[i * octree_size].center;
            Vector3 sizeOct = custOctree[i * octree_size].size;

            // Lv2, Split to 8 children [0-7] 
            custOctree[i * octree_size + 1].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 1].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 1].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 2].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 2].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 2].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 3].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 3].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 3].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 4].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 4].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 4].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 5].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 5].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 5].center.z = centerOct.z + (sizeOct.z / 4);

            custOctree[i * octree_size + 6].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 6].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 6].center.z = centerOct.z + (sizeOct.z / 4);

            custOctree[i * octree_size + 7].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 7].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 7].center.z = centerOct.z + (sizeOct.z / 4);

            custOctree[i * octree_size + 8].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 8].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 8].center.z = centerOct.z + (sizeOct.z / 4);

            
            for (int j = 1; j <= 8; j++)
            {
                OctreeData oct = custOctree[i * octree_size + j];
                custOctree[i * octree_size + j].min = oct.Minimum(oct.center, (custBB[i].size / 4));
                custOctree[i * octree_size + j].max = oct.Maximum(oct.center, (custBB[i].size / 4));

                // Lv2, Split to 64 children

                custOctree[i * octree_size + j * 8 + 1].center.x = custOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 1].center.y = custOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 1].center.z = custOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 2].center.x = custOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 2].center.y = custOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 2].center.z = custOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 3].center.x = custOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 3].center.y = custOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 3].center.z = custOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 4].center.x = custOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 4].center.y = custOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 4].center.z = custOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 5].center.x = custOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 5].center.y = custOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 5].center.z = custOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 6].center.x = custOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 6].center.y = custOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 6].center.z = custOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 7].center.x = custOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 7].center.y = custOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 7].center.z = custOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 8].center.x = custOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 8].center.y = custOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 8].center.z = custOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                for (int k = 1; k <= 8; k++)
                {
                    custOctree[i * octree_size + j * 8 + k].min = oct.Minimum(custOctree[i * octree_size + j * 8 + k].center, (custBB[i].size / 8));
                    custOctree[i * octree_size + j * 8 + k].max = oct.Maximum(custOctree[i * octree_size + j * 8 + k].center, (custBB[i].size / 8));

                }
            }
        }
    }

    private void CheckCollisionOctreeLv0()
    {
        collidablePairIndexL0.Clear();
        for (int i = 0; i < pairIndexL0.Count; i++)
        {
            if (Intersection.AABB(custOctree[pairIndexL0[i].i1].min, custOctree[pairIndexL0[i].i1].max,
            custOctree[pairIndexL0[i].i2].min, custOctree[pairIndexL0[i].i2].max))
            {
                if (debugModeLv0)
                {
                    if (!collidablePairIndexL0.Contains(pairIndexL0[i].i1))
                        collidablePairIndexL0.Add(pairIndexL0[i].i1);

                    if (!collidablePairIndexL0.Contains(pairIndexL0[i].i2))
                        collidablePairIndexL0.Add(pairIndexL0[i].i2);
                }
            }
        }
    }

    private void CheckCollisionOctreeLv1()
    {
        collidablePairIndexL1.Clear();
        for (int i = 0; i < pairIndexL1.Count; i++)
        {
            int l0_index_obj1 = (int)Mathf.Floor(pairIndexL1[i].i1 / octree_size) * octree_size;
            int l0_index_obj2 = (int)Mathf.Floor(pairIndexL1[i].i2 / octree_size) * octree_size;

            if (Intersection.AABB(custOctree[l0_index_obj1].min, custOctree[l0_index_obj1].max,
                custOctree[l0_index_obj2].min, custOctree[l0_index_obj2].max))
            {
                if (Intersection.AABB(custOctree[pairIndexL1[i].i1].min, custOctree[pairIndexL1[i].i1].max,
                    custOctree[pairIndexL1[i].i2].min, custOctree[pairIndexL1[i].i2].max))
                {
                    if (debugModeLv1)
                    {
                        // use any to map multi pair and not duplicte of Lv1
                        if (!collidablePairIndexL1.Any(c => c.Equals(pairIndexL1[i])))
                            collidablePairIndexL1.Add(pairIndexL1[i]);
                    }
                }
            }
        }
    }

    private int calcIndexObjectLevel0(int object_index)
    {
        int level = (int)Mathf.Floor(object_index / octree_size);


        //if (level == 1) return level * 73;
        if (level != 0) return level * octree_size;
        return (int)Mathf.Floor(object_index / octree_size);
    }

    private int calcIndexObjectLevel1(int object_index )
    {
        int level = (int)Mathf.Floor(object_index / octree_size);

        if (level == 1) return (int)Mathf.Floor((object_index - (1 + level)) / 8) + level * 64;
        if (level == 2) return (int)Mathf.Floor((object_index - (1 + level)) / 8) + level * 64;
        return (int)Mathf.Floor((object_index - 1) / 8) + level * 64 ;
    }

    private void CheckCollisionOctreeLv2()
    {

        collidablePairIndexL2.Clear();
        for (int i = 0; i < pairIndexL2.Count; i++)
        {
            int l0_index_obj1 = calcIndexObjectLevel0(pairIndexL2[i].i1);
            int l0_index_obj2 = calcIndexObjectLevel0(pairIndexL2[i].i2);

            int l1_index_obj1 = calcIndexObjectLevel1(pairIndexL2[i].i1);
            int l1_index_obj2 = calcIndexObjectLevel1(pairIndexL2[i].i2);

            if (Intersection.AABB(custOctree[l0_index_obj1].min, custOctree[l0_index_obj1].max,
                custOctree[l0_index_obj2].min, custOctree[l0_index_obj2].max))
            {
                if (Intersection.AABB(custOctree[l1_index_obj1].min, custOctree[l1_index_obj1].max,
                    custOctree[l1_index_obj2].min, custOctree[l1_index_obj2].max))
                {
                    if (Intersection.AABB(custOctree[pairIndexL2[i].i1].min, custOctree[pairIndexL2[i].i1].max,
                        custOctree[pairIndexL2[i].i2].min, custOctree[pairIndexL2[i].i2].max))
                    {
                        if (debugModeLv2)
                        {
                            if (!collidablePairIndexL2.Any(c => c.Equals(pairIndexL2[i])))
                                collidablePairIndexL2.Add(pairIndexL2[i]);
                        }
                    }

                }
            }
        }
    }

    private void AddTriangles(int i)
    {
        var posTri = deformableCPUMSM[i].GetPosTriangles();
        for(int j=0; j< triCount; j++)
        {
            posTriangle.Add(new Tris
            {
                vertex0 = posTri[j].vertex0,
                vertex1 = posTri[j].vertex1,
                vertex2 = posTri[j].vertex2
            });
        }
    }

    private void CheckTriIntersection() 
    {
        collidableTriIndex.Clear();
        for(int i =0; i< number_object * triCount;i++)
        {
            for(int j =0; j< number_object * triCount;j++)
            {
                int objIndex1 = (int)Mathf.Floor(i/triCount);
                int objIndex2 = (int)Mathf.Floor(j/triCount);
               
                if(objIndex1 != objIndex2 && objIndex1 >= objIndex2)
                {
                    collidablePairIndexL0.Clear();
                    for (int p = 0; p < pairIndexL0.Count; p++)
                    {
                        // check collision box of object, before check tri-intersection
                        if (Intersection.AABB(custOctree[pairIndexL0[p].i1].min, custOctree[pairIndexL0[p].i1].max,
                        custOctree[pairIndexL0[p].i2].min, custOctree[pairIndexL0[p].i2].max))
                        {                               
                            if (debugModeLv0)
                            {
                                if (!collidablePairIndexL0.Contains(pairIndexL0[p].i1))
                                    collidablePairIndexL0.Add(pairIndexL0[p].i1);

                                if (!collidablePairIndexL0.Contains(pairIndexL0[p].i2))
                                    collidablePairIndexL0.Add(pairIndexL0[p].i2);
                            }


                            var t1 = posTriangle[i];
                            var t2 = posTriangle[j];

                            if (Detection(t1, t2))
                            {                   
                                Vector2Int pair = new Vector2Int(i, j);
                                if(!collidableTriIndex.Contains(pair)){
                                    collidableTriIndex.Add(pair); 
                                }

                                
                                // // �浹 ������ �ﰢ���� �� ���� ���� ����� �̿��Ͽ� �̵� ���� ���
                                // Vector3 collisionPoint = hitPoint; // �浹 ����

                                // // �浹 ������ ���� ����� ������ ã��
                                // {
                                //     Vector3 closestVertex = FindClosestVertex(t1, collisionPoint);
                                //     Vector3 averageVel = t1.getAverageVelocity();

                                //     Vector3 separationVector = (closestVertex - collisionPoint).normalized * separationDistance;
                                //     if (Vector3.Distance(averageVel, Vector3.zero) > 0.00001)
                                //     {
                                //         //// �̵� ���͸� ����Ͽ� �浹�� �߻��� ������ �浹 �������� �̵���Ŵ
                                //         if (closestVertex == t1.vertex0)
                                //         {
                                //             t1.vertex0 -= (t1.vel0 * t1.deltaTime) + separationVector;
                                //             t1.vel0 *= -1.0f;
                                //         }
                                //         else if (closestVertex == t1.vertex1)
                                //         {
                                //             t1.vertex1 -= (t1.vel1 * t1.deltaTime) + separationVector;
                                //             t1.vel1 *= -1.0f;
                                //         }
                                //         else if (closestVertex == t1.vertex2)
                                //         {
                                //             t1.vertex2 -= (t1.vel2 * t1.deltaTime) + separationVector;
                                //             t1.vel2 *= -1.0f;
                                //         }
                                //     }
                                // }
                                // {
                                //     Vector3 closestVertex = FindClosestVertex(t2, collisionPoint);
                                //     Vector3 averageVel = t2.getAverageVelocity();

                                //     if (Vector3.Distance(averageVel, Vector3.zero) > 0.00001)
                                //     {
                                //         Vector3 separationVector = (closestVertex - collisionPoint).normalized * separationDistance;

                                //         //// �̵� ���͸� ����Ͽ� �浹�� �߻��� ������ �浹 �������� �̵���Ŵ
                                //         if (closestVertex == t2.vertex0)
                                //         {
                                //             t2.vertex0 -= (t2.vel0 * t2.deltaTime * 2.0f) + separationVector;
                                //             t2.vel0 *= -1.0f;
                                //         }
                                //         else if (closestVertex == t2.vertex1)
                                //         {
                                //             t2.vertex1 -= (t2.vel1 * t2.deltaTime * 2.0f) + separationVector;
                                //             t2.vel1 *= -1.0f;
                                //         }
                                //         else if (closestVertex == t2.vertex2)
                                //         {
                                //             t2.vertex2 -= (t2.vel2 * t2.deltaTime * 2.0f) + separationVector;
                                //             t2.vel2 *= -1.0f;
                                //         }
                                //     }
                                // }
                                // posTriangle[i] = t1;
                                // posTriangle[j] = t2;
                            }
                            //UpdatePosition();
                        }
                    }                        
                    
                }
            }
        }  
    }

    bool Detection(Tris t1, Tris t2)
    {
        var c1 = CheckEdgeCollision(t1.vertex0, t1.vertex1, t2) || 
        CheckEdgeCollision(t1.vertex0, t1.vertex2, t2) || 
        CheckEdgeCollision(t1.vertex1, t1.vertex2, t2);

        var c2 = CheckEdgeCollision(t2.vertex0, t2.vertex1, t1) || 
        CheckEdgeCollision(t2.vertex0, t2.vertex2, t1) || 
        CheckEdgeCollision(t2.vertex1, t2.vertex2, t1);

        return c1 && c2;
    }


    Vector3 ProjectPointOnPlane(Vector3 point, Vector3 planeNormal, Vector3 planePoint)
    {
        float d = Vector3.Dot(planeNormal, (point - planePoint)) / planeNormal.magnitude;
        return point - d * planeNormal;
    }


    bool IsPointInsideTriangle(Vector3 point, Tris triangle)
    {
         Vector3 normal = Vector3.Cross(triangle.vertex1 - triangle.vertex0, triangle.vertex2 - triangle.vertex0).normalized;

        // ���� �ﰢ�� ��鿡 ����
        Vector3 projectedPoint = ProjectPointOnPlane(point, normal, triangle.vertex0);

        if (Vector3.Distance(projectedPoint, point) > 0.1) return false;

        //Debug.Log(Vector3.Distance(projectedPoint, point));

        // ������ ���� ���� ���� �Ǵ� ����
        Vector3 edge1 = triangle.vertex1 - triangle.vertex0;
        Vector3 vp1 = projectedPoint - triangle.vertex0;
        if (Vector3.Dot(Vector3.Cross(edge1, vp1), normal) < 0) return false;

        Vector3 edge2 = triangle.vertex2 - triangle.vertex1;
        Vector3 vp2 = projectedPoint - triangle.vertex1;
        if (Vector3.Dot(Vector3.Cross(edge2, vp2), normal) < 0) return false;

        Vector3 edge3 = triangle.vertex0 - triangle.vertex2;
        Vector3 vp3 = projectedPoint - triangle.vertex2;
        if (Vector3.Dot(Vector3.Cross(edge3, vp3), normal) < 0) return false;

        return true; // ��� �˻縦 ����ߴٸ�, ������ ���� �ﰢ�� ���ο� �ֽ��ϴ�.
    }

    Vector3 FindClosestVertex(Tris triangle, Vector3 point)
    {
        float minDistance = Mathf.Infinity;
        Vector3 closestVertex = Vector3.zero;

        float distance0 = Vector3.Distance(triangle.vertex0, point);
        float distance1 = Vector3.Distance(triangle.vertex1, point);
        float distance2 = Vector3.Distance(triangle.vertex2, point);

        if (distance0 < minDistance)
        {
            minDistance = distance0;
            closestVertex = triangle.vertex0;
        }
        if (distance1 < minDistance)
        {
            minDistance = distance1;
            closestVertex = triangle.vertex1;
        }
        if (distance2 < minDistance)
        {
            minDistance = distance2;
            closestVertex = triangle.vertex2;
        }

        return closestVertex;
    }

    bool checkPointCollision(Vector3 p, Tris triangle)
    {
        return IsPointInsideTriangle(p, triangle);
    }

    bool CheckEdgeCollision(Vector3 vertex1, Vector3 vertex2, Tris t)
    {
        var edge = new Line();

        edge.p0 = vertex1;
        edge.p1 = vertex2;

        return Intersect(t, edge, ref hitPoint);
    }

    public bool Intersect(Tris triangle, Line ray, ref Vector3 hit)
    {
        // Vectors from p1 to p2/p3 (edges)
        //Find vectors for edges sharing vertex/point p1
        Vector3 e1 = triangle.vertex1 - triangle.vertex0;
        Vector3 e2 = triangle.vertex2 - triangle.vertex0;

        // Calculate determinant
        Vector3 p = Vector3.Cross(ray.direction, e2);

        //Calculate determinat
        float det = Vector3.Dot(e1, p);

        //if determinant is near zero, ray lies in plane of triangle otherwise not
        if (det > -Mathf.Epsilon && det < Mathf.Epsilon)
        {
            var coplanar = IsPointInsideTriangle(ray.p0, triangle);
            var coplanar2 = IsPointInsideTriangle(ray.p1, triangle);

            if (coplanar) hit = ray.p0;
            if (coplanar2) hit = ray.p1;

            return coplanar || coplanar2;
        }
        float invDet = 1.0f / det;

        //calculate distance from p1 to ray origin
        Vector3 t = ray.origin - triangle.vertex0;

        //Calculate u parameter
        float u = Vector3.Dot(t, p) * invDet;

        //Check for ray hit
        if (u < 0 || u > 1) { return false; }

        //Prepare to test v parameter
        Vector3 q = Vector3.Cross(t, e1);

        //Calculate v parameter
        float v = Vector3.Dot(ray.direction, q) * invDet;

        //Check for ray hit
        if (v < 0 || u + v > 1) { return false; }

        // intersection point
        hit = triangle.vertex0 + u * e1 + v * e2;

        if ((Vector3.Dot(e2, q) * invDet) > Mathf.Epsilon)
        {
            //ray does intersect            
            return true;
        }

        // No hit at all
        return false;
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
        if (custOctree != null )
        {
            for (int i = 0; i < custOctree.Length; i++)
            {
                if (debugModeLv0)
                {
                    Gizmos.color = Color.red;
                    for (int j = 0; j < collidablePairIndexL0.Count; j++)
                    {
                        if (i == collidablePairIndexL0[j])
                        {
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL0[j]].center, custOctree[collidablePairIndexL0[j]].size);
                        }
                    }
                }

                if (debugModeLv1)
                {
                    Gizmos.color = Color.green;
                    for (int j = 0; j < collidablePairIndexL1.Count; j++)
                    {
                        if (i == collidablePairIndexL1[j].i1)
                        {
                            Vector3 size = custOctree[collidablePairIndexL1[j].i1].max - custOctree[collidablePairIndexL1[j].i1].min;
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL1[j].i1].center, size);
                        }
                        if (i == collidablePairIndexL1[j].i2)
                        {
                            Vector3 size = custOctree[collidablePairIndexL1[j].i2].max - custOctree[collidablePairIndexL1[j].i2].min;
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL1[j].i2].center, size);
                        }
                    }
                }

                if (debugModeLv2)
                {
                    Gizmos.color = Color.blue;
                    for (int j = 0; j < collidablePairIndexL2.Count; j++)
                    {
                       
                        if (i == collidablePairIndexL2[j].i1)
                        {
                            Vector3 size = custOctree[collidablePairIndexL2[j].i1].max - custOctree[collidablePairIndexL2[j].i1].min;
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL2[j].i1].center, size);
                        }
                        if (i == collidablePairIndexL2[j].i2)
                        {
                            Vector3 size = custOctree[collidablePairIndexL2[j].i2].max - custOctree[collidablePairIndexL2[j].i2].min;
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL2[j].i2].center, size);
                        }
                    }
                }
            }
        }

        if (collidableTriIndex != null)
        {
            List<int> newList = new List<int>();
            for(int i =0; i< collidableTriIndex.Count; i++){
                if(!newList.Contains(collidableTriIndex[i].x)) newList.Add(collidableTriIndex[i].x);
                if(!newList.Contains(collidableTriIndex[i].y)) newList.Add(collidableTriIndex[i].y);
            }

            for(int i =0; i< newList.Count; i++)
            {
                DrawTriangle(posTriangle[newList[i]], Color.green);
            }
        }
    }

     private void DrawTriangle(Tris triangle, Color color)
    {
        Gizmos.color = color;
        Gizmos.DrawLine(triangle.vertex0, triangle.vertex1);
        Gizmos.DrawLine(triangle.vertex1, triangle.vertex2);
        Gizmos.DrawLine(triangle.vertex2, triangle.vertex0);
    }
}
