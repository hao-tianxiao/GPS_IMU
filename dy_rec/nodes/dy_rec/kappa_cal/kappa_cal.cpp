#include <math.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
using namespace std;

string in_path="/home/ttzn/ros_ins570/src/dy_rec/nodes/dy_rec/path.txt";
string out_path="/home/ttzn/ros_ins570/src/dy_rec/nodes/dy_rec/kappa_path.txt";

double distance(double a[2],double b[2])//求两点间距离，数组a[2]为点a的坐标信息，a[0]为a的x坐标，a[1]为a的y坐标
{
     double dis;//两点间距离
     double x,y,x2,y2;
     x=a[0]-b[0];
     y=a[1]-b[1];
     x2=x*x;
     y2=y*y;
     dis=sqrt((x2+y2));//double sqrt(double x)为求平方根函数
     return dis;   
}
int collinear(double a[2],double b[2],double c[2])//判断三点是否共线，共线返回1
{
      double k1,k2;
      double kx1,ky1,kx2,ky2;
      if(a[0]==b[0]&&b[0]==c[0])  return 1;//三点横坐标都相等，共线
      else
        {
          kx1=b[0]-a[0];
          kx2=b[0]-c[0];
          ky1=b[1]-a[1];
          ky2=b[1]-a[1];
          k1=ky1/kx1;
          k2=ky2/kx2;
          if(k1==k2) return 1;//AB与BC斜率相等，共线
           else  return 0;//不共线
         }
}
double curvature(double a[2],double b[2],double c[2])//double为数据类型，
{                                                    //数组a[2]为点a的坐标信息，a[0]为a的x坐标，a[1]为a的y坐标
       double cur;//求得的曲率
       if(collinear(a,b,c)==1)//判断三点是否共线
       {
        cur=0.0;//三点共线时将曲率设为某个值，0
        }
       else
      {
       double radius;//曲率半径
       double dis,dis1,dis2,dis3;//距离
       double cosA;//ab确定的边所对应的角A的cos值
       dis1=distance(a,c);
       dis2=distance(a,b);
       dis3=distance(b,c);
       dis=dis1*dis1-(dis2*dis2+dis3*dis3);
       cosA=dis/(2*dis2*dis3);//余弦定理
       radius=0.5*dis1/(1-pow(cosA,2));
       cur=1/radius;
      }
       return cur; 
}

int main()
{
    double p[3][3];
    ifstream in_f;
    ofstream out_f;
    in_f.open(in_path);
    out_f.open(out_path,ios::trunc);
    int c = 2; //the last point
    for(int i =0; i<2; i++)
            in_f>>p[i][0]>>p[i][1]>>p[i][2];
    while(!in_f.eof())
    {
        in_f>>p[c][0]>>p[c][1]>>p[c][2];
        cout<<p[c][0]<<p[c][1]<<endl;
        int c_1 = c-1==-1?2:c-1;
        out_f<<p[(c_1)%3][0]<<" "<<p[(c_1)%3][1]<<" "<<p[(c_1)%3][2]<<" "<<curvature(p[(c+1)%3],p[(c+2)%3],p[c%3])<<endl;
        c=(c+1)%3;
    }
    in_f.close();
    out_f.close();
    return 0;
}