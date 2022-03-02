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
  private void RunScript(Curve boundary, Curve shape_type, double floor_height, int floor_num, double density_min, double density_max, double plotRatio_min, double plotRatio_max, double threashold, double scale_factor, double area_limit, int floor_min, int floor_max, ref object position, ref object buildings, ref object offShape, ref object w)
  {
    //======初始化======
    com_void = new Common_void();
    string warning = "拟合";
    //计算场地面积
    double boundaryArea = AreaMassProperties.Compute(boundary).Area;
    double shape_area = AreaMassProperties.Compute(shape_type).Area;

    //建筑密度计算智能体个数范围
    double area_scale = shape_area;
    int scale_dnum = 0;
    while(area_scale >= area_limit){
      area_scale = area_scale * scale_factor * scale_factor;
      scale_dnum += 1;
    }
    double area_min = area_scale * 1 / (scale_factor * scale_factor);
    scale_dnum -= 1;

    int build_num_dmin = (int) Math.Ceiling(boundaryArea * density_min / shape_area);
    int build_num_dmax = (int) (boundaryArea * density_max / (area_min));
    //容积率计算智能体个数范围
    int build_num_pmin = (int) Math.Ceiling(plotRatio_min * boundaryArea / (shape_area * floor_max));
    int build_num_pmax = (int) (plotRatio_max * boundaryArea / (area_min * floor_min));
    //综合计算智能体个数范围
    int build_num_min,build_num_max = 0;
    if(build_num_dmin > build_num_pmin){
      build_num_min = build_num_dmin;}
    else{
      build_num_min = build_num_pmin;
    }
    if(build_num_dmax > build_num_pmax){
      build_num_max = build_num_pmax;}
    else{
      build_num_max = build_num_dmax;
    }
    //生成随机智能体个数
    Random rd = new Random();
    int build_num = rd.Next(build_num_min, build_num_max);
    List<double> tt = new List<double>();
    //tt.Add(shape_area);
    //tt.Add(area_min);
    //tt.Add(build_num);
    //生成相应个数的随机点与基于模板的建筑图形
    List<Point3d> randomPoints = com_void.GetRandomPoints(build_num, boundary);
    List< Curve> shape_types = new List<Curve>();
    shape_types = com_void.RepeatCurve(build_num, shape_type);
    //创建智能体
    bs = new List<Building>();
    for(int i = 0; i < build_num; i++){
      Building b = new Building(boundary, randomPoints[i], shape_types[i], floor_height, floor_num, scale_dnum);
      bs.Add(b);
    }

    double density = double.MinValue;
    double plotRatio = double.MinValue;
    bool ifChangeScale = true;
    bool ifLimitCount = false;
    double count = 0;
    double totalOverlap = double.MaxValue;
    double scale_anum = 0;
    List<int> seq = new List<int>();
    //======运行======
    //跳出循环条件
    //totalOverlap <= shape_area * threashold
    //density_min<=density <= density_max
    //plotRatio_min<=plotRatio <= plotRatio_max
    while(!(totalOverlap <= shape_area * threashold && density >= density_min && density <= density_max && plotRatio >= plotRatio_min && plotRatio <= plotRatio_max)){
      totalOverlap = 0;
      //迭代次数大于100时，迭代次数为智能体个数时对最不符合要求的建筑缩小
      if(ifChangeScale){
        if(count > 50 && count % bs.Count == 0){
          //索引
          seq.Clear();
          seq = new List<int>();
          for(int i = 0;i < bs.Count ;i++){
            //计算各智能体不符合要求的面积总和——>逾越场地边界的面积及其与其他智能体缓冲区内的总重叠面积
            bs[i].UpdateOverlap(bs);
            //获取缩小未完全达到限制的智能体索引
            if(bs[i].alimit != true){
              seq.Add(i);
            }
          }
          //获取总面积从大到小的相应索引
          int temp = 0;
          for(int i = 0;i < seq.Count - 1;i++){
            for(int j = 0;j < seq.Count - i - 1;j++){
              if(bs[seq[j]].overlapArea < bs[seq[j + 1]].overlapArea){
                temp = seq[j];
                seq[j] = seq[j + 1];
                seq[j + 1] = temp;
              }
            }
          }
          //缩小直至碰到一个可缩小的智能体，则退出循环
          for(int i = 0;i < seq.Count;i++){
            bool bool_atemp = bs[seq[i]].Narrow(scale_factor);
            if(bool_atemp)
              break;
          }
          //清空各智能体的存储数据——>不符合要求的面积总和
          for(int i = 0;i < bs.Count ;i++){
            bs[i].ClearOverlapArea();
          }
        }
      }

      for(int i = 0;i < bs.Count;i++){
        //计算场地边界对出界智能体的动量
        bs[i].CheckEdge();
        //根据该智能体与其他智能体的缓冲边界内的重叠情况，计算动量
        for(int j = 0; j < bs.Count; j++){
          if(i != j){
            bs[i].Seperate(bs[j]);
          }
        }
        //根据动量更新智能体位置
        bs[i].Update();
        //清空各智能体的存储数据——>不符合要求的面积总和
        bs[i].ClearOverlapArea();
      }
      if(scale_dnum > 0)
        scale_anum = com_void.GetScaleANum(bs);
      else
        scale_anum = 0;
      totalOverlap = com_void.CheckOverlap_bs(bs);
      count++;

      if(count == 100 && totalOverlap > shape_area * threashold && ifLimitCount){
        build_num = bs.Count - 1;
        bs.Clear();
        shape_types.Clear();
        randomPoints.Clear();
        randomPoints = com_void.GetRandomPoints(build_num, boundary);
        shape_types = com_void.RepeatCurve(build_num, shape_type);
        for(int i = 0; i < build_num; i++){
          Building b = new Building(boundary, randomPoints[i], shape_types[i], floor_height, floor_num, scale_dnum);
          bs.Add(b);
        }
        //直接进入下一次循环
        ifChangeScale = true;
        ifLimitCount = false;
        totalOverlap = double.MaxValue;
        scale_anum = 0;
        count = 0;
        continue;
      }
      //计算建筑密度与容积率
      double value_atemp = 0;
      double value_ftemp = 0;
      for(int i = 0;i < bs.Count;i++)
      {
        value_atemp += AreaMassProperties.Compute(bs[i].shape).Area;
        value_ftemp += bs[i].floor_num * AreaMassProperties.Compute(bs[i].shape).Area;
      }
      density = value_atemp / boundaryArea;
      plotRatio = value_ftemp / boundaryArea;

      //拟合失败，并所有已缩小至极致
      if(totalOverlap > shape_area * threashold && scale_anum == bs.Count && count % bs.Count == 0){
        build_num = bs.Count - 1;
      }
      //拟合成功,但建筑密度小于下限
      if(totalOverlap <= shape_area * threashold && density < density_min){
        build_num = bs.Count + 1;
      }
      //超出范围，退出循环
      if(build_num > build_num_max || build_num < build_num_min){
        warning = "拟合失败，原因可能是参数无法满足拟合，请适当放宽参数";
        break;
      }
        //在范围内，且智能体个数需要改变
      else if(build_num != bs.Count){
        bs.Clear();
        shape_types.Clear();
        randomPoints.Clear();
        randomPoints = com_void.GetRandomPoints(build_num, boundary);
        shape_types = com_void.RepeatCurve(build_num, shape_type);
        for(int i = 0; i < build_num; i++){
          Building b = new Building(boundary, randomPoints[i], shape_types[i], floor_height, floor_num, scale_dnum);
          bs.Add(b);
        }
        ifChangeScale = true;
        ifLimitCount = false;
        totalOverlap = double.MaxValue;
        scale_anum = 0;
        count = 0;
        continue;
      }
        //在范围内，且智能体个数不需要改变，则进行容积率的拟合
      else{
        if(totalOverlap <= shape_area * threashold){
          //容积率不在范围内
          if(plotRatio > plotRatio_max || plotRatio < plotRatio_max){
            seq.Clear();
            seq = new List<int>();
            for(int i = 0;i < bs.Count ;i++){
              seq.Add(i);
            }
            //获取从北到南的相应索引
            int temp = 0;
            for(int i = 0;i < seq.Count - 1;i++){
              for(int j = 0;j < seq.Count - i - 1;j++){
                if(bs[seq[j]].pos.Y < bs[seq[j + 1]].pos.Y){
                  temp = seq[j];
                  seq[j] = seq[j + 1];
                  seq[j + 1] = temp;
                }
              }
            }
            //容积率大于上限
            if(plotRatio > plotRatio_max){
              for(int i = floor_num;i > floor_min;i--){
                for(int j = 0;j < bs.Count;j++){
                  bs[seq[bs.Count - j - 1]].floor_num -= 1;
                  value_ftemp -= AreaMassProperties.Compute(bs[seq[bs.Count - j - 1]].shape).Area;
                  if (value_ftemp <= plotRatio_max * boundaryArea){
                    break;
                  }
                }
                if (value_ftemp <= plotRatio_max * boundaryArea){
                  break;
                }
              }
              //在范围内对楼层向下调整后，可使容积率拟合
              if(value_ftemp <= plotRatio_max * boundaryArea){
                ifChangeScale = false;
                ifLimitCount = false;
                totalOverlap = double.MaxValue;
                scale_anum = 0;
                count = 0;
                continue;
              }
                //楼层调至最低仍无法拟合容积率
              else{
                //减少智能体个数
                build_num = bs.Count - 1;
              }
            }
              //容积率小于下限
            else if(plotRatio < plotRatio_min){
              for(int i = floor_num;i < floor_max;i++){
                for(int j = 0;j < bs.Count;j++){
                  bs[seq[j]].floor_num += 1;
                  value_ftemp += AreaMassProperties.Compute(bs[seq[j]].shape).Area;
                  if (value_ftemp >= plotRatio_min * boundaryArea){
                    break;
                  }
                }
                if (value_ftemp >= plotRatio_min * boundaryArea){
                  break;
                }
              }
              //向上调整必然要相应地缩小
              //在范围内对楼层向上调整后，可使容积率拟合
              if(value_ftemp >= plotRatio_min * boundaryArea){
                ifChangeScale = false;
                ifLimitCount = true;
                totalOverlap = double.MaxValue;
                scale_anum = 0;
                count = 0;
                continue;
              }
                //楼层调至最低仍无法拟合容积率
              else{
                //增加智能体个数
                build_num = bs.Count - 1;
              }
            }
              //容积率在范围内
            else{warning = "拟合成功";}
            //超出范围，退出循环
            if(build_num > build_num_max || build_num < build_num_min){
              warning = "拟合失败，原因可能是参数无法满足拟合，请适当放宽参数";
              break;
            }
              //在范围内，且智能体个数需要改变
            else if(build_num != bs.Count){
              bs.Clear();
              shape_types.Clear();
              randomPoints.Clear();
              randomPoints = com_void.GetRandomPoints(build_num, boundary);
              shape_types = com_void.RepeatCurve(build_num, shape_type);
              for(int i = 0; i < build_num; i++){
                Building b = new Building(boundary, randomPoints[i], shape_types[i], floor_height, floor_num, scale_dnum);
                bs.Add(b);
              }
              //直接进入下一次循环
              ifChangeScale = true;
              ifLimitCount = false;
              totalOverlap = double.MaxValue;
              scale_anum = 0;
              count = 0;
              continue;
            }
          }
        }
      }
    }

    //====输出=====
    pos_list = new List<Point3d>();
    footPrint = new List<Curve>();
    offSet = new List<Curve>();


    for(int i = 0; i < bs.Count; i++){
      pos_list.Add(bs[i].pos);
      footPrint.Add(bs[i].shape);
      offSet.Add(bs[i].offShape);
      tt.Add(AreaMassProperties.Compute(bs[i].shape).Area);
      tt.Add(bs[i].floor_num);
    }
    //tt.Add(com_void.CheckOverlap_bs(bs));
    //tt.Add(bs.Count);
    //tt.Add(build_num);
    //tt.Add(count);
    position = pos_list;
    buildings = footPrint;
    offShape = offSet;
    //w = tt;
    w = warning;


  }

  // <Custom additional code> 
  //全局变量
  List<Building> bs;
  Common_void com_void;
  List<Point3d> pos_list;
  List<Curve> footPrint;
  List<Curve> offSet;


  //以下为构建建筑单体智能体代码
  public class Building{
    public Curve boundary;
    public Point3d pos;
    public Curve shape;
    public double floor_height;
    public int floor_num;
    public int scale_dnum;

    public Point3d siteCenter;
    public Curve offShape;
    public Vector3d move;

    public double overlapArea;
    public bool alimit;
    public Random rand;

    public Building(Curve boundary_, Point3d pos_, Curve shape_, double floor_height_, int floor_num_, int scale_dnum_){

      pos = new Point3d();
      floor_height = new double();
      floor_num = new int();
      scale_dnum = new int();
      boundary = boundary_;
      pos = pos_;
      shape = shape_;
      floor_height = floor_height_;
      floor_num = floor_num_;
      scale_dnum = scale_dnum_;

      siteCenter = GetCenter(boundary);
      Point3d center = AreaMassProperties.Compute(shape).Centroid;
      shape.Translate(new Vector3d(pos.X - center.X, pos.Y - center.Y, 0));
      offShape = CreateBuffer(shape, floor_height * floor_num);

      overlapArea = 0;
      if(scale_dnum > 0){
        alimit = false;
      }
      else{alimit = true;}
      rand = new Random();

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
    public void CalculateHeight(){
      offShape = CreateBuffer(shape, floor_height * floor_num);
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
      //找出最北面的边的长度
      double distance_N_len = 0;
      for (int i = 0;i < points_num - 1;i++){
        Point3d temp = new Point3d();
        //南北侧
        if(Math.Abs(points_o[i + 1].X - points_o[i].X) > Math.Abs(points_o[i + 1].Y - points_o[i].Y))
        {
          //北侧
          if((points_o[i + 1].X - points_o[i].X) > 0)
          {
            if(points_o[i + 1].X - points_o[i].X > distance_N_len){
              distance_N_len = points_o[i + 1].X - points_o[i].X;
            }
          }
        }
      }
      //图形均基于顺时针构建，因而可根据两点所形成的矢量方向，判断需要构建哪一方向的缓冲，最后用两点位置的移动进行缓冲的构建
      for (int i = 0;i < points_num - 1;i++){
        Point3d temp = new Point3d();
        //南北侧
        if(Math.Abs(points_o[i + 1].X - points_o[i].X) > Math.Abs(points_o[i + 1].Y - points_o[i].Y))
        {
          //北侧
          if((points_o[i + 1].X - points_o[i].X) > 0)
          {
            if(points_o[i + 1].X - points_o[i].X == distance_N_len){
              distance_N = L;
            }
            else{
              distance_N = 0;
            }
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

    //计算场地边界对出界智能体的动量
    public void CheckEdge(){
      Curve[] differC = Curve.CreateBooleanDifference(shape, boundary, 0.000001);
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
      Curve[] intersecs = Curve.CreateBooleanIntersection(shape, b.offShape, 0.000001);
      if(intersecs.Length != 0){
        for (int j = 0; j < intersecs.Length; j++){
          overlap += AreaMassProperties.Compute(intersecs[j]).Area;
          Point3d interCenter = GetCenter(intersecs[j]);
          Vector3d pi = Point3d.Subtract(pos, interCenter);
          Vector3d pj = Point3d.Subtract(GetCenter(b.offShape), interCenter);
          if(pi.Length == 0){
            pi = Point3d.Subtract(pos, b.pos);
            pj = Point3d.Subtract(b.pos, pos);
          }

          double dist = pos.DistanceTo(GetCenter(b.offShape));
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
    public bool Narrow (double scale_factor){
      if(scale_dnum > 0)
      {
        shape.Transform(Transform.Scale(pos, scale_factor));
        offShape = CreateBuffer(shape, floor_height * floor_num);
        scale_dnum -= 1;
      }
      if(scale_dnum == 0)
      {
        alimit = true;
      }
      return(!alimit);
    }

    //计算各智能体不符合要求面积——>逾越场地边界的面积与其他智能体缓冲区内的重叠面积
    public void UpdateOverlap (List<Building> buildings){
      overlapArea = 0;
      for(int i = 0; i < buildings.Count; i++){
        Building b = buildings[i];
        Curve[] intersecs = Curve.CreateBooleanIntersection(shape, b.offShape, 0.000001);
        if(intersecs.Length != 0){
          for (int j = 0; j < intersecs.Length; j++){
            overlapArea += AreaMassProperties.Compute(intersecs[j]).Area;
          }
        }
      }
      Curve[] differC = Curve.CreateBooleanDifference(shape, boundary, 0.000001);
      if(differC.Length != 0){
        for (int i = 0; i < differC.Length; i++){
          overlapArea += AreaMassProperties.Compute(differC[i]).Area;
        }
      }
    }
  }


  //常用方法
  public class Common_void{
    public Common_void(){}

    //计算所有智能体不符合要求面积总和——>逾越场地边界的面积与其他智能体缓冲区内的重叠面积
    public double CheckOverlap_bs (List<Building> bs){
      double totalOverlap = 0;
      for(int i = 0; i < bs.Count; i++){
        for(int j = 0; j < bs.Count; j++){
          if(i != j)
          {
            Curve[] intersecs = Curve.CreateBooleanIntersection(bs[i].shape, bs[j].offShape, 0.000001);
            if(intersecs.Length != 0){
              for (int k = 0; k < intersecs.Length; k++){
                totalOverlap += AreaMassProperties.Compute(intersecs[k]).Area;
              }
            }
          }
        }
        Curve[] differC = Curve.CreateBooleanDifference(bs[i].shape, bs[i].boundary, 0.000001);
        if(differC.Length != 0){
          for (int j = 0; j < differC.Length; j++){
            totalOverlap += AreaMassProperties.Compute(differC[j]).Area;
          }
        }
      }
      return totalOverlap;
    }


    public int GetScaleANum(List<Building> bs)
    {
      int result = 0;
      for(int i = 0;i < bs.Count;i++)
      {
        if(bs[i].alimit == true)
        {
          result += 1;
        }
      }
      return result;
    }
    //Calculate total area of curves
    public double CalculateTotalArea(List<Curve> curve)
    {
      double result = 0;
      for(int i = 0; i < curve.Count; i++)
      {
        result += AreaMassProperties.Compute(curve[i]).Area;
      }
      return result;
    }

    public List<Point3d>GetRandomPoints(int i, Curve curve)
    {
      var func_info = Rhino.NodeInCode.Components.FindComponent("PopulateGeometry");
      var func = func_info.Delegate as dynamic;
      List<Curve> curves = new List<Curve>();
      curves.Add(curve);
      var result = func(Brep.CreatePlanarBreps(curves), i, 1, null);
      List<Point3d> points = new List<Point3d>();
      foreach (var temp in result)
      {
        points.Add((Point3d) temp);
      }
      return points;
    }
    public List<Curve>RepeatCurve(int i, Curve curve)
    {
      var func_info = Rhino.NodeInCode.Components.FindComponent("RepeatData");
      var func = func_info.Delegate as dynamic;
      var result = func(curve, i);

      List<Curve> curves = new List<Curve>();
      foreach (var temp in result)
      {
        curves.Add((Curve) temp);
      }
      return curves;
    }

  }






  // </Custom additional code> 
}