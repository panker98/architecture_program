using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;



/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance : GH_ScriptInstance
{
#region Utility functions
  /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
  /// <param name="text">String to print.</param>
  private void Print(string text) { /* Implementation hidden. */ }
  /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
  /// <param name="format">String format.</param>
  /// <param name="args">Formatting parameters.</param>
  private void Print(string format, params object[] args) { /* Implementation hidden. */ }
  /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj) { /* Implementation hidden. */ }
  /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj, string method_name) { /* Implementation hidden. */ }
#endregion

#region Members
  /// <summary>Gets the current Rhino document.</summary>
  private readonly RhinoDoc RhinoDocument;
  /// <summary>Gets the Grasshopper document that owns this script.</summary>
  private readonly GH_Document GrasshopperDocument;
  /// <summary>Gets the Grasshopper script component that owns this script.</summary>
  private readonly IGH_Component Component;
  /// <summary>
  /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
  /// Any subsequent call within the same solution will increment the Iteration count.
  /// </summary>
  private readonly int Iteration;
#endregion

  /// <summary>
  /// This procedure contains the user code. Input parameters are provided as regular arguments,
  /// Output parameters as ref arguments. You don't have to assign output parameters,
  /// they will have a default value.
  /// </summary>
  private void RunScript(Curve boundary, List<Curve> roads, List<Curve> shape_type, int num, List<Point3d> RandomPoints, double density, double FAR, double floorHeight, double threashold, bool ifChangeScale, double scale_factor, ref object position, ref object buildings, ref object offShape, ref object closest, ref object levels)
  {
    //======初始化======
    //计算智能体的平均面积
    double boundaryArea = AreaMassProperties.Compute(boundary).Area;
    averageArea = boundaryArea * density / num;
    //创建智能体
    bs = new List<Building>();
    for(int i = 0; i < num; i++){
      Building b = new Building(i, RandomPoints[i], averageArea, boundary, shape_type[i], floorHeight);
      bs.Add(b);
    }
    //计算平均楼层数
    buildArea = AreaMassProperties.Compute(boundary).Area * FAR;
    meanLevel = (int) Math.Ceiling(buildArea / boundaryArea * density);

    count = 0;
    totalOverlap = double.MaxValue;
    //======运行======
    while(totalOverlap > averageArea * threashold){
      totalOverlap = 0;
      //迭代次数大于100时，每迭代5次对最不符合要求的建筑缩小
      if(ifChangeScale && count > 100 && count % 5 == 0){
        //根据智能体逾越场地边界的面积及其与其他智能体缓冲区内的总重叠面积——>判断最不符合要求的建筑
        int scale_index = -1;
        double scale_index_area = 0;
        for(int i = 0;i < num ;i++){
          //计算各智能体不符合要求的面积总和
          bs[i].UpdateOverlap(bs);
          //获取索引
          if(bs[i].overlapArea > scale_index_area)
          {
            scale_index = i;
            scale_index_area = bs[i].overlapArea;
          }
          //清空各智能体的存储数据——>不符合要求的面积总和
          bs[i].ClearOverlapArea();
        }
        //根据索引调用缩放函数
        if(scale_index != -1){
          bs[scale_index].Narrow(scale_factor);
        }
      }

      //计算各智能体与道路距离最短原则，寻找相应的道路点与距离
      for (int i = 0; i < bs.Count; i++){
        double minDist = double.MaxValue;
        int index = -1;
        for (int j = 0; j < roads.Count; j++){
          double t;
          roads[j].ClosestPoint(bs[i].pos, out t);
          Point3d closePoint = roads[j].PointAt(t);
          double dist = bs[i].pos.DistanceTo(closePoint);
          if(dist < minDist){
            minDist = dist;
            index = j;
            bs[i].closestBP = closePoint;
          }
        }
        bs[i].closestDist = minDist;
        bs[i].closestRoad = index;
      }

      //计算当前分布标准差
      double meanDist = 0;
      for (int i = 0; i < bs.Count; i++){
        meanDist += bs[i].closestDist;
      }
      meanDist /= bs.Count;
      double variance = 0;
      for (int i = 0; i < bs.Count; i++){
        variance += (bs[i].closestDist - meanDist) * (bs[i].closestDist - meanDist);
      }
      variance /= bs.Count;
      double standDeviation = Math.Sqrt(variance);
      double lowDist = meanDist - 2 * standDeviation;
      double upDist = meanDist + 2 * standDeviation;
      for(int i = 0; i < num; i++){
        bs[i].Overlap(bs);
      }


      for(int i = 0;i < num ;i++){
        //计算各智能体的楼层数量
        bs[i].CalculateHeight(meanLevel, meanDist, standDeviation, floorHeight, ref bs[i].level);
        //计算场地边界对出界智能体的动量
        bs[i].CheckEdge();
        //根据该智能体与其他智能体的缓冲边界内的重叠情况，计算动量
        for(int j = i + 1; j < num; j++){
          bs[i].Seperate(bs[j]);
        }
        //根据动量更新智能体位置
        bs[i].Update();
        //清空各智能体的存储数据——>不符合要求的面积总和
        bs[i].ClearOverlapArea();
      }
      totalOverlap = bs[0].CheckOverlap(bs);
      count++;
    }

    //====输出=====
    pos = new List<Point3d>();
    footPrint = new List<Curve>();
    offSet = new List<Curve>();
    List < Vector3d > direction = new List<Vector3d>();
    List < Point3d > closestPoint = new List<Point3d>();
    List < double > ratios = new List<double>();
    List < double> level = new List<double>();
    for(int i = 0; i < num; i++){
      footPrint.Add(bs[i].shape);
      pos.Add(bs[i].pos);
      offSet.Add(bs[i].offShape);
      closestPoint.Add(bs[i].closestBP);
      level.Add(bs[i].level);
    }
    level.Add(totalOverlap);
    position = pos;
    buildings = footPrint;
    offShape = offSet;
    closest = closestPoint;
    levels = level;

  }

  // <Custom additional code> 
  //全局变量
  List<Building> bs;
  List<Point3d> pos;
  List<Curve> footPrint;
  List<Curve> offSet;
  int count;
  double totalOverlap;
  double averageArea;
  double buildArea;
  int meanLevel;

  //以下为构建建筑单体智能体代码
  public class Building{
    public Point3d pos;
    public double averageArea;
    public Curve boundary;
    public Curve shape;
    public Curve offShape;

    public double area;
    public double overlapArea;

    public double closestDist;
    public Point3d closestBP;
    public int closestRoad;

    public int level;
    public double height;
    public double floorHeight;

    public Vector3d move;
    public Point3d siteCenter;
    public BoundingBox overlapBox;

    public int index;
    public Random rand;

    public Building(int index_, Point3d pos_, double averageArea_, Curve boundary_, Curve shape_type_, double floorHeight_){
      pos = pos_;
      averageArea = averageArea_;
      boundary = boundary_;
      shape = shape_type_;
      siteCenter = GetCenter(boundary);

      level = 4;
      floorHeight = floorHeight_;
      Point3d center = AreaMassProperties.Compute(shape).Centroid;
      shape.Translate(new Vector3d(pos.X - center.X, pos.Y - center.Y, 0));
      shape.Transform(Transform.Scale(pos, Math.Sqrt(averageArea / AreaMassProperties.Compute(shape).Area)));
      offShape = CreateBuffer(shape, level * 3);

      area = AreaMassProperties.Compute(shape).Area;
      overlapArea = 0;

    }

    //计算图形几何中心
    private Point3d GetCenter(Curve c){
      Point3d center = AreaMassProperties.Compute(c).Centroid;
      return center;
    }

    //清空各智能体的存储数据——>不符合要求的面积总和
    public void ClearOverlapArea(){
      overlapArea = 0;
    }

    //计算各智能体楼层数&&创建缓冲
    public void CalculateHeight(int meanLevel, double meanDist, double standDeviation, double floorHeight, ref int level){
      double deviation = closestDist - meanDist;
      if(Math.Abs(deviation) < 0.5 * standDeviation){
        level = meanLevel;
      }
      if(deviation >= 0.5 * standDeviation && deviation < 1.5 * standDeviation){
        level = (int) (meanLevel * 0.75);
      }
      if(deviation <= -0.5 * standDeviation && deviation > -1.5 * standDeviation){
        level = (int) (meanLevel * 1.25);
      }
      if(deviation <= -1.5 * standDeviation){
        level = (int) (meanLevel * 1.5);
      }
      if(deviation >= 1.5 * standDeviation){
        level = (int) (meanLevel * 0.5);
      }
      offShape = CreateBuffer(shape, level * floorHeight);
    }
    //创建缓冲
    private Curve CreateBuffer(Curve x, double y)
    {

      List<Point3d> points_o = new List<Point3d>();
      List<Point3d> points_c = new List<Point3d>();
      int points_num = x.ToNurbsCurve().Points.Count;
      for (int i = 0;i < points_num;i++)
      {
        Point3d temp = new Point3d();
        x.ToNurbsCurve().Points.GetPoint(i, out temp);
        points_o.Add(temp);
        points_c.Add(temp);
      }
      //计算东南西北侧方向上的缓冲距离
      double distance_N,distance_E,distance_S,distance_W;
      if(0.3 * 30 + y - 30 * 0.1 < 6)
      {
        distance_E = distance_W = 6;
      }
      else
      {
        distance_E = distance_W = 0.3 * 30 + y - 30 * 0.1;
      }
      /*
      if(0.8 * 30 + y - 30 * 0.5 < 8)
      {
        distance_S = 8;
      }
      else
      {
        distance_S = 0.8 * 30 + y - 30 * 0.5;
      }
      */
      distance_S = 0;
      double sinHs = Math.Sin(Math.PI / 180 * 23) * Math.Sin(Math.PI / 180 * -23.5) + Math.Cos(Math.PI / 180 * 23) * Math.Cos(Math.PI / 180 * -23.5) * Math.Cos(0);
      double L = Math.Sqrt((y / sinHs) * (y / sinHs) - y * y);
      distance_N = L;
      //图形均基于顺时针构建，因而可根据两点所形成的矢量方向，判断需要构建哪一方向的缓冲，最后用两点位置的移动进行缓冲的构建
      for (int i = 0;i < points_num - 1;i++)
      {
        Point3d temp = new Point3d();
        //南北侧
        if(Math.Abs(points_o[i + 1].X - points_o[i].X) > Math.Abs(points_o[i + 1].Y - points_o[i].Y))
        {
          //北侧
          if((points_o[i + 1].X - points_o[i].X) > 0)
          {
            temp = new Point3d(points_c[i].X, points_c[i].Y + distance_N, points_c[i].Z);
            points_c.RemoveAt(i);
            points_c.Insert(i, temp);
            temp = new Point3d(points_c[i + 1].X, points_c[i + 1].Y + distance_N, points_c[i + 1].Z);
            points_c.RemoveAt(i + 1);
            points_c.Insert(i + 1, temp);
          }
            //南侧
          else
          {
            temp = new Point3d(points_c[i].X, points_c[i].Y - distance_S, points_c[i].Z);
            points_c.RemoveAt(i);
            points_c.Insert(i, temp);
            temp = new Point3d(points_c[i + 1].X, points_c[i + 1].Y - distance_S, points_c[i + 1].Z);
            points_c.RemoveAt(i + 1);
            points_c.Insert(i + 1, temp);
          }
        }
          //东西侧
        else
        {
          if((points_o[i + 1].Y - points_o[i].Y) < 0)
          {
            //东侧
            temp = new Point3d(points_c[i].X + distance_E, points_c[i].Y, points_c[i].Z);
            points_c.RemoveAt(i);
            points_c.Insert(i, temp);
            temp = new Point3d(points_c[i + 1].X + distance_E, points_c[i + 1].Y, points_c[i + 1].Z);
            points_c.RemoveAt(i + 1);
            points_c.Insert(i + 1, temp);
          }
          else
          {
            //西侧
            temp = new Point3d(points_c[i].X - distance_W, points_c[i].Y, points_c[i].Z);
            points_c.RemoveAt(i);
            points_c.Insert(i, temp);
            temp = new Point3d(points_c[i + 1].X - distance_W, points_c[i + 1].Y, points_c[i + 1].Z);
            points_c.RemoveAt(i + 1);
            points_c.Insert(i + 1, temp);
          }
        }
      }
      //计算一个新的重合点，因为建筑轮廓上第一个点与最后一个点重合，构建缓冲后不重合
      double temp_x,temp_y;
      if(points_o[0].X == points_c[0].X)
      {
        temp_x = points_c[points_num - 1].X;
        temp_y = points_c[0].Y;
      }
      else
      {
        temp_x = points_c[0].X;
        temp_y = points_c[points_num - 1].Y;
      }
      Point3d point_orignal = new Point3d(temp_x, temp_y, points_c[0].Z);
      //重合点替换第一点
      points_c.RemoveAt(0);
      points_c.Insert(0, point_orignal);
      //重合点替换最后一点
      points_c.RemoveAt(points_num - 1);
      points_c.Insert(points_num - 1, point_orignal);
      Polyline buffer = new Polyline(points_c);
      return buffer.ToNurbsCurve();
    }


    //避免中心点重叠
    public void Overlap(List<Building> buildings){
      Vector3d randMove = new Vector3d(0, 0, 0);
      for(int i = 0; i < buildings.Count; i++){
        Building b = buildings[i];
        if(b != this && pos == b.pos){
          double randDir = Math.PI * rand.NextDouble();
          double bArea = AreaMassProperties.Compute(b.offShape).Area; //面积越大移动越远
          randMove = new Vector3d(Math.Cos(randDir) * bArea * 0.005, Math.Sin(randDir) * bArea * 0.005, 0);
          break;
        }
      }
      pos = Point3d.Add(pos, randMove);
      shape.Translate(randMove);
      offShape.Translate(randMove);
    }

    //计算各智能体不符合要求面积——>逾越场地边界的面积与其他智能体缓冲区内的重叠面积
    public void UpdateOverlap (List<Building> buildings){
      overlapArea = 0;
      for(int i = 0; i < buildings.Count; i++){
        Building b = buildings[i];
        Curve[] intersecs = Curve.CreateBooleanIntersection(offShape, b.shape, 0);
        if(intersecs.Length != 0){
          for (int j = 0; j < intersecs.Length; j++){
            overlapArea += AreaMassProperties.Compute(intersecs[j]).Area;
          }
        }
      }
      Curve[] differC = Curve.CreateBooleanDifference(shape, boundary, 0);
      if(differC.Length != 0){
        for (int i = 0; i < differC.Length; i++){
          overlapArea += AreaMassProperties.Compute(differC[i]).Area;
        }
      }
    }

    //计算所有智能体不符合要求面积总和——>逾越场地边界的面积与其他智能体缓冲区内的重叠面积
    public double CheckOverlap (List<Building> bs){
      double totalOverlap = 0;
      for(int i = 0; i < bs.Count; i++){
        for(int j = i + 1; j < bs.Count; j++){
          Curve[] intersecs = Curve.CreateBooleanIntersection(bs[i].offShape, bs[j].shape, 0);
          if(intersecs.Length != 0){
            for (int k = 0; k < intersecs.Length; k++){
              totalOverlap += AreaMassProperties.Compute(intersecs[k]).Area;
            }
          }
        }
        Curve[] differC = Curve.CreateBooleanDifference(bs[i].shape, bs[i].boundary, 0);
        if(differC.Length != 0){
          for (int j = 0; j < differC.Length; j++){
            totalOverlap += AreaMassProperties.Compute(differC[j]).Area;
          }
        }
      }
      return totalOverlap;
    }

    //计算场地边界对出界智能体的动量
    public void CheckEdge(){
      Curve[] differC = Curve.CreateBooleanDifference(shape, boundary, 0);
      if(differC.Length != 0){
        Vector3d pushBack = new Vector3d();
        for (int i = 0; i < differC.Length; i++){
          overlapArea += AreaMassProperties.Compute(differC[i]).Area;
          Point3d subCenter = GetCenter(differC[i]);
          Vector3d push = Point3d.Subtract(siteCenter, subCenter);
          pushBack = Vector3d.Add(pushBack, push);
        }
        pushBack = Vector3d.Multiply(pushBack, 2.5 / pushBack.Length);
        move += pushBack;
      }
    }

    //判断该智能体是否与某智能体缓冲区内重叠，若有则两智能体都增加动量
    public void Seperate (Building b){
      double overlap = 0;
      Curve[] intersecs = Curve.CreateBooleanIntersection(offShape, b.shape, 0);
      if(intersecs.Length != 0){
        for (int j = 0; j < intersecs.Length; j++){
          overlap += AreaMassProperties.Compute(intersecs[j]).Area;
          Point3d interCenter = GetCenter(intersecs[j]);
          Vector3d pi = Point3d.Subtract(pos, interCenter);
          Vector3d pj = Point3d.Subtract(b.pos, interCenter);
          if(pi.Length == 0){
            pi = Point3d.Subtract(pos, b.pos);
            pj = Point3d.Subtract(b.pos, pos);
          }

          double dist = pos.DistanceTo(b.pos);
          pi = Vector3d.Multiply(pi, 4 / dist);
          pj = Vector3d.Multiply(pj, 4 / dist);
          move = Vector3d.Add(move, Vector3d.Multiply(pi, 0.5));
          b.move = Vector3d.Add(b.move, Vector3d.Multiply(pj, 0.5));
        }
      }
    }

    //根据动量更新智能体位置
    public void Update (){
      pos = Point3d.Add(pos, move);
      shape.Translate(move);
      offShape.Translate(move);
      move = Vector3d.Multiply(move, 0);
    }

    //缩小
    public void Narrow (double scale_factor){
      shape.Transform(Transform.Scale(pos, scale_factor));
      offShape = CreateBuffer(shape, level * floorHeight);
    }

  }



  // </Custom additional code> 
}